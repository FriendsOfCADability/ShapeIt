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
        if (methodName.EndsWith("get_Surface1") || methodName.EndsWith("get_Surface2")) return; // would cause recursion
        if (instance is InterpolatedDualSurfaceCurve ipdsc)
        {
            if (ipdsc.Surface1.IsUPeriodic && ipdsc.bounds1.Width > ipdsc.Surface1.UPeriod ||
                ipdsc.Surface1.IsVPeriodic && ipdsc.bounds1.Height > ipdsc.Surface1.VPeriod ||
                ipdsc.Surface2.IsUPeriodic && ipdsc.bounds2.Width > ipdsc.Surface2.UPeriod ||
                ipdsc.Surface2.IsVPeriodic && ipdsc.bounds2.Height > ipdsc.Surface2.VPeriod)
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