# ShapeIt.Profiler

Runs one RPC case headless and repeatedly, so a profiler has something worth looking at.

Profiling the application itself rarely works: the message loop, the rendering and the idle time drown
out the geometry, and the interesting work happens inside user interactions that are never repeated
identically. This host runs one case, no window, no message loop.

The case is executed through the very same harness the RPC regression tests use
(`tests/CADability.Tests/RPC/RpcRunner.cs`, linked into this project rather than copied), so what is
measured here is what the tests run.

## Usage

```
ShapeIt.Profiler <case> [-n <count>] [-w <count>] [-s <mb>] [--wait]
ShapeIt.Profiler --list
```

| option | meaning |
|---|---|
| `<case>` | path to a `*.json` case, or the bare name of one in `tests\CADability.Tests\Files\RPC` |
| `-n <count>` | measured iterations, default 5 |
| `-w <count>` | warmup iterations, excluded from the result, default 1 |
| `-s <mb>` | stack size of the worker thread, default 32 MB |
| `--wait` | run the warmup, then wait for Enter, so a profiler can be attached and started exactly around the measured iterations |
| `--list` | list the cases that can be named without a path |

```
ShapeIt.Profiler DieWithPips -n 10
```

Build it with MSBuild, not with `dotnet build`: `ShapeIt.csproj` bumps its version through a
`CodeTaskFactory` task, and that factory does not exist in the .NET Core version of MSBuild.

## Reading the report

```
wall ms   min 4434   median 4502   mean 4492   max 4540
cpu ms    mean 4390  (98% of wall - CPU bound, a CPU profile is meaningful)
alloc     6.37 GB per iteration
gc        gen0 1537,5   gen1 58,0   gen2 3,5   per iteration
```

* **cpu vs wall** decides which tool can answer the question at all. Near 100 %: the run is computing,
  and a CPU profile points at the cause. Clearly below: the run is waiting - GC, locks, I/O - and a CPU
  sampler will point at whatever happened to be running instead. Note that a CPU-capped environment (a
  sandbox, a container with a CPU quota, a busy machine) depresses this number without anything being
  wrong with the code; read it from a normal console on an otherwise idle machine.
* **alloc per iteration** is often the real cost in geometry code. Gigabytes per run means the time goes
  into allocating and collecting, not into arithmetic, and an allocation profile will be more useful
  than a CPU profile.
* **per call** breaks the run down by top level RPC call before any profiler is involved. That is
  usually enough to know which operation to look at.

## With a profiler

The `--wait` switch exists so the profiler only sees the measured iterations: start the host, let it do
its warmup, attach, then press Enter.

**PerfView** is the best first choice here, because it takes managed method names from the CLR's own ETW
events and therefore does not depend on PDBs at all:

```
PerfView.exe /Providers=*ShapeIt-Profiler run ShapeIt.Profiler.exe DieWithPips -n 5
```

The `ShapeIt-Profiler` provider is this project's own `EventSource` (see `ProfilerEvents.cs`). Its
events mark case, warmup and measured iterations on the timeline, so a time range can be restricted to
"iteration 3" instead of being guessed from the shape of the CPU graph. Then open *CPU Stacks* for the
hot path and *GC Heap Alloc Ignore Free* for the allocations.

**dotnet-trace** produces a flame graph without installing anything heavy:

```
dotnet-trace collect --providers ShapeIt-Profiler -- ShapeIt.Profiler.exe DieWithPips -n 5
```

and `dotnet-trace convert --format speedscope` turns the result into something https://speedscope.app
will display.

**Visual Studio** works too, but its CPU Usage view hides everything it cannot attribute to source
behind a single `[External Code]` node - switch *Show External Code* on, or turn off *Just My Code*
under Tools → Options → Debugging.

## Runtime knobs worth knowing

* `DOTNET_TieredCompilation=0` makes everything compile fully optimized right away. Profiles get easier
  to read (no tier 0 frames, no OSR), at the price of no longer matching how the application starts.
* `DOTNET_gcServer=1` switches to server GC. Worth trying when the report shows the run is allocation
  bound - but it changes the measurement, so do not compare a server GC run against a workstation GC
  one.
