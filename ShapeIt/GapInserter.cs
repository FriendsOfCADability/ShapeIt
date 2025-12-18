using System;
using System.Collections.Generic;

public static class GapInserter
{
    /// <summary>
    /// Fügt Werte an den größten Lücken einer aufsteigend sortierten Liste ein,
    /// bis Count == targetCount ist. Optional können äußere Grenzen mitbetrachtet werden.
    /// </summary>
    public static void FillLargestGaps(
        List<double> xs,
        int targetCount,
        double? leftBound = null,
        double? rightBound = null,
        double eps = 1e-12)
    {
        if (xs == null) throw new ArgumentNullException(nameof(xs));
        if (targetCount <= xs.Count) return;
        xs.Sort(); // zur Sicherheit

        // Hilfs-Record für Intervalle
        var pq = new PriorityQueue<(double a, double b), double>(); // min-heap nach Priority
        void PushInterval(double a, double b)
        {
            double gap = b - a;
            if (double.IsNaN(gap) || double.IsInfinity(gap) || gap <= eps) return;
            // Wir wollen Max-Heap -> negative Priorität
            pq.Enqueue((a, b), -gap);
        }

        // Start-Intervalle: zwischen allen Nachbarn
        for (int i = 0; i + 1 < xs.Count; i++)
            PushInterval(xs[i], xs[i + 1]);

        // Optional: äußere Intervalle mit Grenzen berücksichtigen
        if (leftBound.HasValue)
            PushInterval(leftBound.Value, xs.Count > 0 ? xs[0] : rightBound ?? leftBound.Value);

        if (rightBound.HasValue && xs.Count > 0)
            PushInterval(xs[^1], rightBound.Value);

        // Sonderfall: xs war leer, aber beide Grenzen vorhanden
        if (xs.Count == 0 && leftBound.HasValue && rightBound.HasValue)
        {
            // so oft halbieren, bis targetCount erreicht ist
            while (xs.Count < targetCount && pq.Count > 0)
                InsertMidpointAndSplit(xs, pq, eps);
            return;
        }

        // Hauptschleife: größte Lücke nehmen, Mittelpunkt einfügen, splitten
        while (xs.Count < targetCount && pq.Count > 0)
            InsertMidpointAndSplit(xs, pq, eps);
    }

    private static void InsertMidpointAndSplit(
        List<double> xs,
        PriorityQueue<(double a, double b), double> pq,
        double eps)
    {
        if (!pq.TryDequeue(out var interval, out _)) return;
        double a = interval.a, b = interval.b;
        double mid = 0.5 * (a + b);

        // Robust gegen degenerierte Intervalle
        if (!(b - a > eps) || double.IsNaN(mid) || double.IsInfinity(mid))
            return;

        // sortierte Einfügung per BinarySearch
        int idx = xs.BinarySearch(mid);
        if (idx < 0) idx = ~idx;

        // Falls Numerik nahe an bestehendem Wert: überspringen
        if (idx > 0 && Math.Abs(xs[idx - 1] - mid) <= eps) return;
        if (idx < xs.Count && Math.Abs(xs[idx] - mid) <= eps) return;

        xs.Insert(idx, mid);

        // Neue Intervalle (a, mid) und (mid, b) zurück in die Queue
        double gapLeft = mid - a;
        double gapRight = b - mid;

        if (gapLeft > eps) pq.Enqueue((a, mid), -gapLeft);
        if (gapRight > eps) pq.Enqueue((mid, b), -gapRight);
    }
}
