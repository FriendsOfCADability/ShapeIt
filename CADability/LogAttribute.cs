using MethodDecorator.Fody.Interfaces;
using System;
using System.Reflection;

[AttributeUsage(AttributeTargets.Method | AttributeTargets.Constructor | AttributeTargets.Class)]
public class LogAttribute : Attribute, IMethodDecorator
{
    private string _methodName;

    // Wird vor der Methode ausgeführt
    public void OnEntry()
    {
        System.Diagnostics.Trace.WriteLine($"→ Enter {_methodName}");
    }

    // Wird nach erfolgreichem Ende ausgeführt
    public void OnExit()
    {
        System.Diagnostics.Trace.WriteLine($"← Exit {_methodName}");
    }

    // Wird ausgeführt, wenn die Methode eine Exception wirft
    public void OnException(Exception exception)
    {
        System.Diagnostics.Trace.WriteLine($"‼ Exception in {_methodName}: {exception.Message}");
    }

    // Wird einmal beim Laden (mit Methodendaten) aufgerufen
    public void Init(object instance, MethodBase method, object[] args)
    {
        _methodName = $"{method.DeclaringType.Name}.{method.Name}";
    }
}
