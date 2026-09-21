using System.Diagnostics;
using System.Reflection;

namespace CADability.Tests
{
    /// <summary>
    /// The regression baselines (Files/BRep, Files/RPC, Files/STL) are recorded from a Release build of CADability,
    /// which is the default configuration of this test project (see Directory.Build.props). A Debug build computes
    /// some results differently in the last few digits, because a couple of #if DEBUG blocks have side effects. A
    /// baseline written from Debug would therefore flip back and forth with the configuration of whoever regenerated
    /// it last, and comparing the output of two runs digit for digit - the check for a refactoring that must not
    /// change anything - would become useless. The harnesses ask here before they write a baseline.
    /// </summary>
    internal static class BuildConfiguration
    {
        /// <summary>True when CADability.dll was built without optimization, i.e. in Debug.</summary>
        public static bool CADabilityIsDebug { get; } =
            typeof(CADability.GeoPoint).Assembly.GetCustomAttribute<DebuggableAttribute>()?.IsJITOptimizerDisabled == true;

        /// <summary>Fails the calling test when <paramref name="what"/> is about to be written from a Debug build.</summary>
        public static void RequireReleaseForBaselines(string what)
        {
            if (CADabilityIsDebug)
                Assert.Fail($"refusing to write {what} from a Debug build: the baselines are recorded from Release. "
                    + "Run \"dotnet test tests/CADability.Tests/CADability.Tests.csproj\" without -c, Release is the default.");
        }
    }
}
