# CADability.Tests

Run the tests from the command line:

```bash
dotnet test tests/CADability.Tests/CADability.Tests.csproj
```

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
