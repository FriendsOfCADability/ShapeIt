using System;
using System.Runtime.CompilerServices;

namespace CADability
{
    /// <summary>
    /// Process wide configuration of MathNet.Numerics, applied by a module initializer so that every host gets
    /// it without having to know about it: the WinForms application, the Avalonia and Browser heads, the MCP
    /// server and the regression suites alike.
    /// <para>
    /// MathNet's managed provider parallelizes its matrix operations. The problems it is handed here are tiny -
    /// the Levenberg-Marquardt solvers behind <c>BoxedSurfaceExtension.SurfacesIntersectionLM</c> and
    /// <c>PositionOfLM</c> fit a handful of parameters each - and distributing those over threads costs more
    /// than solving them. Measured over the whole RPC regression set: 12 percent less wall clock time, 40
    /// percent less CPU and 9 percent less allocation, with every recorded baseline unchanged.
    /// </para>
    /// <para>
    /// It also keeps the debugger usable, which is why it was introduced: with parallelization on, evaluating
    /// an expression in the debugger fails with "Cannot evaluate expression since the function evaluation
    /// requires all threads to run."
    /// </para>
    /// <para>
    /// This used to live in <c>MainForm</c> under <c>#if DEBUG</c>, where it reached neither the shipped
    /// release build, nor any head other than WinForms, nor the tests - so the configuration that was measured
    /// and the one that was shipped were never the same one.
    /// </para>
    /// </summary>
    internal static class NumericsConfiguration
    {
        /// <summary>
        /// Runs once, before any other code of this assembly. MathNet is reached through CADability from every
        /// host, so this is early enough for all of them without any of them having to call anything.
        /// </summary>
        [ModuleInitializer]
        internal static void Apply()
        {
            MathNet.Numerics.Control.MaxDegreeOfParallelism = 1;
        }
    }
}

#if !NET5_0_OR_GREATER
namespace System.Runtime.CompilerServices
{
    /// <summary>
    /// The attribute the compiler looks for when emitting a module initializer. It ships with .NET 5 and
    /// later; this assembly targets netstandard2.0, where declaring it here is the supported way to get one.
    /// </summary>
    [AttributeUsage(AttributeTargets.Method, Inherited = false)]
    internal sealed class ModuleInitializerAttribute : Attribute
    {
    }
}
#endif
