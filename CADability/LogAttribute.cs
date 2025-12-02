using MethodDecorator.Fody.Interfaces;
using System;
using System.Reflection;
using CADability;
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
        if (instance is ProjectedCurve pc)
        {
            if (pc.startPoint2d == GeoPoint2D.Origin && pc.endPoint2d == GeoPoint2D.Origin)
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