// Roslyn-Version deiner Scripting-Klasse
using System;
using System.Collections;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using System.Threading;
using System.Collections.Generic;

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CADability
{
    internal class ScriptingException : ApplicationException
    {
        public ScriptingException(string msg) : base(msg) { }
    }

    /// <summary>
    /// Roslyn-basierte Laufzeitkompilierung als Ersatz für CSharpCodeProvider.
    /// API-kompatibel zu deiner bisherigen Scripting-Klasse.
    /// </summary>
    internal class Scripting
    {
        // optional: Konfig
        private static readonly CSharpCompilationOptions CompilationOpts =
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary, optimizationLevel: OptimizationLevel.Release);

        public Scripting()
        {
            // keine Initialisierung wie bei CodeDom nötig
        }

        private static IEnumerable<MetadataReference> GetDefaultReferences()
        {
            // Robuste Referenz-Ermittlung: alles, was schon geladen ist und einen Pfad hat
            // + ausdrücklich wichtige Assemblies (mscorlib/System.Private.CoreLib, System.Runtime, Collections, CADability)
            var loaded = AppDomain.CurrentDomain.GetAssemblies()
                .Where(a => !a.IsDynamic && !string.IsNullOrEmpty(a.Location))
                .Select(a => a.Location)
                .Distinct();

            var refs = new HashSet<string>(loaded, StringComparer.OrdinalIgnoreCase);

            void Add(Assembly a)
            {
                if (!a.IsDynamic && !string.IsNullOrEmpty(a.Location)) refs.Add(a.Location);
            }

            // Basis
            Add(typeof(object).Assembly);                // System.Private.CoreLib / mscorlib
            Add(typeof(Enumerable).Assembly);            // System.Linq
            Add(typeof(Dictionary<string,object>).Assembly);             // System.Collections
            Add(typeof(Regex).Assembly);                 // System.Text.RegularExpressions

            // CADability selbst (GeoPoint/GeoVector liegen hier)
            Add(typeof(GeoPoint).Assembly);

            return refs.Select(p => MetadataReference.CreateFromFile(p));
        }

        private object GetValue(NamedValuesProperty namedValues, string typename, string formula)
        {
            // Dein vorhandener Fix für Ganzzahldivision beibehalten:
            if (Settings.GlobalSettings.GetBoolValue("Scripting.ForceFloat", false))
            {
                formula = Regex.Replace(formula, @"(?<=/)(\d+)\b(?!\.)", "$1.0"); // "1/2" -> "1/2.0"
            }

            string code = @"
using System;
using System.Collections;
using CADability;

public class ScriptClass
{
    double sin(double d) { return Math.Sin(d); }
    double cos(double d) { return Math.Cos(d); }
    double tan(double d) { return Math.Tan(d); }
    double Sin(double d) { return Math.Sin(d/180*Math.PI); }
    double Cos(double d) { return Math.Cos(d/180*Math.PI); }
    double Tan(double d) { return Math.Tan(d/180*Math.PI); }
    GeoVector v(double x, double y, double z) { return new GeoVector(x,y,z); }
    GeoPoint p(double x, double y, double z) { return new GeoPoint(x,y,z); }
    %namedValues%
    Dictionary<string,object> namedValues;
    public ScriptClass(Dictionary<string,object> namedValues)
    {
        this.namedValues = namedValues;
    }
    public %type% Calculate()
    {
        return %formula%;
    }
}
";
            code = code.Replace("%formula%", formula);
            code = code.Replace("%type%", typename);
            code = code.Replace("%namedValues%", namedValues.GetCode()); // z.B. GeoPoint p1 { get { return (GeoPoint)namedValues["p1"]; } }

            // Roslyn: Parsen + Kompilieren ins Memory-Stream
            var syntax = CSharpSyntaxTree.ParseText(code, new CSharpParseOptions(LanguageVersion.Preview));
            var compilation = CSharpCompilation.Create(
                assemblyName: "CADability_Script_" + Guid.NewGuid().ToString("N"),
                syntaxTrees: new[] { syntax },
                references: GetDefaultReferences(),
                options: CompilationOpts
            );

            using var peStream = new MemoryStream();
            var emitResult = compilation.Emit(peStream);

            if (!emitResult.Success)
            {
                // Optional: detaillierte Fehlermeldung zusammenfassen
                var diag = string.Join(Environment.NewLine, emitResult.Diagnostics
                    .Where(d => d.Severity == DiagnosticSeverity.Error)
                    .Select(d => d.ToString()));
                throw new ScriptingException("Compile error: " + diag);
            }

            peStream.Position = 0;
            Assembly generatedAssembly;
            try
            {
                generatedAssembly = Assembly.Load(peStream.ToArray());
            }
            catch (Exception)
            {
                throw new ScriptingException("Assembly Load error");
            }

            try
            {
                var scriptType = generatedAssembly.GetType("ScriptClass", throwOnError: true);
                var ctor = scriptType.GetConstructor(new Type[] { typeof(Dictionary<string, object>) });
                if (ctor == null) throw new ScriptingException("Constructor not found");

                var scriptInstance = ctor.Invoke(new object[] { namedValues.Table });
                var mi = scriptType.GetMethod("Calculate", BindingFlags.Instance | BindingFlags.Public);
                if (mi == null) throw new ScriptingException("Calculate() not found");

                try
                {
                    return mi.Invoke(scriptInstance, null);
                }
                catch (TargetInvocationException)
                {
                    throw new ScriptingException("General error");
                }
            }
            catch (Exception e)
            {
                if (e is ThreadAbortException) throw;
                throw new ScriptingException("General error");
            }
        }

        public GeoVector GetGeoVector(NamedValuesProperty namedValues, string formula)
        {
            return (GeoVector)GetValue(namedValues, "GeoVector", formula);
        }

        public GeoPoint GetGeoPoint(NamedValuesProperty namedValues, string formula)
        {
            return (GeoPoint)GetValue(namedValues, "GeoPoint", formula);
        }

        public double GetDouble(NamedValuesProperty namedValues, string formula)
        {
            return (double)GetValue(namedValues, "double", formula);
        }
    }
}
