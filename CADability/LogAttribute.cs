using MethodDecorator.Fody.Interfaces;
using System;
using System.Reflection;
using CADability;
using CADability.GeoObject;
#if DEBUG
[AttributeUsage(AttributeTargets.Method | AttributeTargets.Constructor | AttributeTargets.Class)]
public class LogAttribute : Attribute, IMethodDecorator
{
    private string methodName;
    private object instance;

    // Wird vor der Methode ausgeführt
    public void OnEntry()
    {
    }

    // Wird nach erfolgreichem Ende ausgeführt
    public void OnExit()
    {
        // if (methodName.EndsWith("get_Surface1") || methodName.EndsWith("get_Surface2")) return; // would cause recursion
        if (instance is ISurfaceImpl si)
        {
            if (Math.Abs(si.usedArea.Left - 3.1191820062517275) < 1e-6)
            {

            }
        }
    }

    // Wird ausgeführt, wenn die Methode eine Exception wirft
    public void OnException(Exception exception)
    {
        System.Diagnostics.Trace.WriteLine($"‼ Exception in {methodName}: {exception.Message}");
    }

    // Wird einmal beim Laden (mit Methodendaten) aufgerufen
    public void Init(object instance, MethodBase method, object[] args)
    {
        this.instance = instance;
        methodName = $"{method.DeclaringType.Name}.{method.Name}";
    }
}
#endif