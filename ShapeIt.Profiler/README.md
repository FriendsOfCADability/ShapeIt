# ShapeIt.Profiler

Runs RPC cases headless and repeatedly, so a profiler has something worth looking at: one case measured in
detail, or a whole set of them to see where the time of the regression suite actually sits.

Profiling the application itself rarely works: the message loop, the rendering and the idle time drown
out the geometry, and the interesting work happens inside user interactions that are never repeated
identically. This host runs one case, no window, no message loop.

The case is executed through the very same harness the RPC regression tests use
(`tests/CADability.Tests/RPC/RpcRunner.cs`, linked into this project rather than copied), so what is
measured here is what the tests run.

## Usage

```
ShapeIt.Profiler <case>... [-n <count>] [-w <count>] [-s <mb>] [--wait]
ShapeIt.Profiler --all      [-n <count>] [-w <count>] [-s <mb>] [--wait]
ShapeIt.Profiler --list
```

| option | meaning |
|---|---|
| `<case>` | path to a `*.json` case, or the bare name of one in `tests\CADability.Tests\Files\RPC`. More than one may be named |
| `--all` | every case of that directory; the ones marked `Skip` and the unusable ones are reported and left out |
| `-n <count>` | measured iterations, default 5 for a single case and 1 for a set |
| `-w <count>` | warmup iterations, excluded from the result, default 1 for a single case and 0 for a set |
| `-s <mb>` | stack size of the worker thread, default 32 MB |
| `--wait` | run the warmup, then wait for Enter, so a profiler can be attached and started exactly around the measured iterations. For a set it waits once, before the first case |
| `--math-threads <n>` | override `MathNet.Numerics.Control.MaxDegreeOfParallelism`, which `CADability.NumericsConfiguration` sets to 1 for every host. `--math-threads 8` measures what the parallelization costs: over the whole set it was 12 % more wall clock time and 40 % more CPU |
| `--list` | list the cases that can be named without a path |

```
ShapeIt.Profiler DieWithPips -n 10
ShapeIt.Profiler --all
```

The defaults differ because the two modes answer different questions. One case is a measurement, and five
iterations show how much it scatters. A set is a ranking, and five iterations of forty cases is an hour.

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

### A set of cases

`--all` replaces the single case report with two tables on one scale, so they can be read against each other:

```
total     wall 512.7 s   cpu 856.1 s (167% - more than one core busy - the surplus is the concurrent GC)

per case (mean of 1 iteration(s), share and running sum of the total)
     54123 ms  10.6%   10.6%  DieWithPips
     ...

per method (mean of 1 iteration(s))
    312456 ms  60.9%     84 call(s)  solid.boolean
     ...
       637 ms   9.9%            project setup and collecting the result
```

* The **running sum** in the case table is what says whether the suite has a few expensive cases or is
  expensive everywhere - the difference decides whether optimizing one algorithm is worth anything.
* The **method table** attributes the same total to the operations of the toolset. This is as far as one gets
  without a profiler, and it is usually enough to know which operation to sample.
* A **CPU share above 100 %** means more than one core was busy. The run itself is single threaded, so the
  surplus is the concurrent GC - a hint that the allocation deserves a look before the arithmetic does.
* With `-w 0`, the default for a set, the **first case also pays for the JIT**. Read the table as a ranking;
  give it `-w 1` when the absolute numbers matter.

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
dotnet-trace collect --profile dotnet-sampled-thread-time --providers ShapeIt-Profiler --format Speedscope -- ShapeIt.Profiler.exe DieWithPips -n 5
dotnet-trace collect --profile dotnet-sampled-thread-time --providers ShapeIt-Profiler --format Speedscope -- ShapeIt.Profiler.exe --all -n 3
```

**`--profile` is not optional here.** `--providers` adds to a profile, it does not select one, so leaving
the profile out collects this project's marker events and no stacks at all - a trace that finishes without
an error and contains nothing to read. On Windows the profile that samples managed stacks is
`dotnet-sampled-thread-time`; `cpu-sampling` exists only for `collect-linux`.

`--format Speedscope` writes a second file next to the nettrace that https://speedscope.app opens directly;
`dotnet-trace convert --format speedscope` does the same afterwards. Sampling runs at roughly 100 Hz, so
give the run enough iterations to collect a few thousand samples - `-n 3` over the whole set is about two
minutes and twelve thousand of them.

The `--all` form is the one that answers "where does the regression suite spend its time": one process,
only geometry, and the `CaseStart`/`CaseStop` events on the timeline to cut the trace down to a single case
afterwards. Do not point a sampler at `dotnet test` instead - it profiles the test host, the JIT of
everything it loads and the baseline comparison along with the geometry.

**Visual Studio** works too, but its CPU Usage view hides everything it cannot attribute to source
behind a single `[External Code]` node - switch *Show External Code* on, or turn off *Just My Code*
under Tools → Options → Debugging.

## Runtime knobs worth knowing

* `DOTNET_TieredCompilation=0` makes everything compile fully optimized right away. Profiles get easier
  to read (no tier 0 frames, no OSR), at the price of no longer matching how the application starts.
* `DOTNET_gcServer=1` switches to server GC. Worth trying when the report shows the run is allocation
  bound - but it changes the measurement, so do not compare a server GC run against a workstation GC
  one.
