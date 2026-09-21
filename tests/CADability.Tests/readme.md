# CADability.Tests

Run the tests from the command line:

```bash
dotnet test tests/CADability.Tests/CADability.Tests.csproj
```

This runs against a **Release** build: `Directory.Build.props` next to the project makes Release the default when
no configuration is given, and hands it on to the referenced projects (CADability, CADability.Forms.NET8,
ShapeIt). Release is what users get, the whole suite takes about 3.5 minutes instead of well over 15, and it is
deterministic from run to run. The regression baselines in `Files/BRep`, `Files/RPC` and `Files/STL` are recorded
from Release, and the harnesses refuse to write one from a Debug build.

`dotnet test ... -c Debug` still works and still matches the baselines within their tolerance. Run it from time to
time anyway: only Debug checks the `Debug.Assert` statements, and the test host turns a failed assertion into a
failed test. Visual Studio's Test Explorer uses the configuration selected in the IDE.

Note that this project is not part of any solution — `CADability.sln` does not contain it, so
`dotnet test CADability.sln` runs nothing.

> Earlier versions of this file claimed that `VsInstallRoot` has to be set. That is not the case: no project
> reads the variable. `CADability.csproj` references `Microsoft.VisualStudio.DebuggerVisualizers` through a
> hard coded relative path into `Program Files\Microsoft Visual Studio\2022\Community\...`, so what it really
> needs is an installed VS 2022 — the variable never had any effect.

# Coverage in Visual Studio

* Recommended extension: Fine Code Coverage
* Settings:
  * Enabled: True
  * RunMsCodeCoverage: True

This will show coverage markers directly in Visual Studio and a Coverage Report in the Fine Code Coverage Tool window.

# Generating Test Artifacts

Recommended for generating cobertura.xml and junit.xml (i.e. for GitLab)

```bash
dotnet test CADability.sln  --collect:"XPlat Code Coverage" --logger:"junit;LogFilePath=test-results.xml;MethodFormat=Class;FailureBodyFormat=Verbose" --settings coverlet.runsettings
```

> ! Does currently not work and test run hangs
