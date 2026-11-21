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

`StepProbe` imports a file with `Project.ReadFromFile(file, "stp")` and dumps the object hierarchy
(type, name, bounding box) into a text file. It references the built
`CADability/bin/Release/netstandard2.0/CADability.dll`.

1. Build CADability in Release with the old state, build StepProbe, and copy its output folder,
   e.g. to `probe_base`.
2. Make the change, build CADability in Release again, build StepProbe again (`probe_new`).
3. Run both over the file list and compare:

```
dotnet build CADability/CADability.csproj -c Release
dotnet build tests/StepCorpus/StepProbe -c Release -o <work>/probe_base
python runcorpus.py <work>/probe_base/StepProbe.exe asmfiles.txt <work>/base base 3 900
python runcorpus.py <work>/probe_new/StepProbe.exe asmfiles.txt <work>/new new 3 900
python compare.py <work>/base base <work>/new new --orientation asmorient.tsv
```

Some imports are not deterministic (the solids are created in parallel and the unit factor is
global), so the same version can give different results in two runs. Import a reported file again
with both versions before blaming the change.
