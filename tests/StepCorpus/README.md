# STEP corpus tools

Tools to check changes of the STEP import (`CADability/ImportStep.cs`) against a large collection of
STEP files. The STEP files themselves are not part of the repository (many are customer files); the
lists and results produced by these tools contain their paths and names, so keep them out of the
repository as well.

Requires Python 3 and the .NET 8 SDK.

## Structure analysis (no CADability involved)

`stepstruct.py` parses the STEP files on its own and reports the product structure, which entities
hold the shapes, units, mapped items, and for every assembly occurrence how the
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION is oriented (rep_1 = child or parent).

```
python stepstruct.py <file.stp>                         # one file, JSON to stdout
python stepstruct.py <filelist> struct.jsonl [workers]  # a list of files (one path per line)
python summarize.py struct.jsonl                        # statistics over the corpus
python list_assemblies.py struct.jsonl asmfiles.txt asmorient.tsv
```

## Comparing two versions of the import

`StepProbe` imports a file with `ImportStep` (like `Project.ReadFromFile` does) and dumps
- the problems reported by the import (`ImportStep.Problems`),
- the object hierarchy (type, name, bounding box), compared by `compare.py`,
- one line per solid and shell (faces, edges, open edges, volume, surface types), compared by
  `geomcompare.py`.

StepProbe references the built `CADability/bin/Release/netstandard2.0/CADability.dll`, not the
project: building StepProbe does not rebuild CADability. Always build CADability first.

### The two scenarios

`makelists.py` searches the corpus and writes two file lists (sorted, so the numbering of
`runcorpus.py` stays the same between runs):

- `quick.txt`: the files smaller than 10 MB (about 850 files, about 1.5 hours with 6 workers).
  This is the regular check of a change of the import or of the geometry.
- `full.txt`: all files. Some large files take many minutes each; run this when there is time,
  e.g. before a release or after a larger change.

```
python makelists.py <corpus root> <work> [--max-mb 10]
```

### Procedure

1. Build the old state and keep the probe:
   ```
   dotnet build CADability/CADability.csproj -c Release
   dotnet build tests/StepCorpus/StepProbe -c Release -o <work>/probe_base
   ```
2. Make the change and build the new probe (`probe_new`) the same way.
3. Run the baseline twice (two labels in one run) and the new state once, then compare:
   ```
   python runcorpus.py <work>/probe_base/StepProbe.exe <work>/quick.txt <work>/base base,base2 6 600
   python runcorpus.py <work>/probe_new/StepProbe.exe <work>/quick.txt <work>/new new 6 600
   python geomcompare.py <work>/base base <work>/new new --noise <work>/base base2
   ```
   For the full scenario use `full.txt` and a larger timeout. The baseline runs can be kept and
   reused as long as the baseline state does not change.

`geomcompare.py` gives every changed file a verdict: `worse` (only deteriorations: exception,
timeout, more problems, fewer solids, more open edges, inward oriented solids), `better`, `mixed`
or `changed` (only neutral changes: faces, volumes, surface types). Files whose two baseline runs
already differ are listed as `noisy`: the import is not deterministic (the solids are created in
parallel and the unit factor is global), in the quick scenario about 6 % of the files differ between
two runs of the same version. Before blaming a change, import a reported file again with both
versions. The report also lists files that became much slower and the totals of the surface types,
e.g. how many NURBS surfaces became analytic surfaces.

`compare.py` compares the hierarchies (names, bounding boxes, assembly orientation):

```
python compare.py <work>/base base <work>/new new --orientation asmorient.tsv
```

### Pitfalls

- Use Release builds only. A Debug CADability.dll is much slower and stops at `Debug.Assert`;
  StepProbe writes `DEBUG` into the header of every result and `geomcompare.py` warns about it.
- Do not run the unit tests at the same time: the tests with time limits fail under the load.
