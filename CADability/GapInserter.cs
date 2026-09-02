using System;
using System.Collections.Generic;

namespace CADability.GeoObject
{
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
            GapHeap pq = new GapHeap(); // max-heap nach Luecken-Groesse
            void PushInterval(double a, double b)
            {
                double gap = b - a;
                if (double.IsNaN(gap) || double.IsInfinity(gap) || gap <= eps) return;
                pq.Enqueue(a, b, gap);
            }

            // Start-Intervalle: zwischen allen Nachbarn
            for (int i = 0; i + 1 < xs.Count; i++)
                PushInterval(xs[i], xs[i + 1]);

            // Optional: äußere Intervalle mit Grenzen berücksichtigen
            if (leftBound.HasValue)
                PushInterval(leftBound.Value, xs.Count > 0 ? xs[0] : rightBound ?? leftBound.Value);

            if (rightBound.HasValue && xs.Count > 0)
                PushInterval(xs[xs.Count - 1], rightBound.Value);

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
            GapHeap pq,
            double eps)
        {
            if (!pq.TryDequeue(out double a, out double b)) return;
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

            if (gapLeft > eps) pq.Enqueue(a, mid, gapLeft);
            if (gapRight > eps) pq.Enqueue(mid, b, gapRight);
        }

        /// <summary>
        /// Binary max heap over intervals, keyed by gap width. Replaces PriorityQueue&lt;,&gt;, which
        /// only exists from .NET 6 on and is therefore unavailable in netstandard2.0. Like
        /// PriorityQueue it does not break ties in any defined order.
        /// </summary>
        private class GapHeap
        {
            private readonly List<(double a, double b, double gap)> items = new List<(double a, double b, double gap)>();

            public int Count => items.Count;

            public void Enqueue(double a, double b, double gap)
            {
                items.Add((a, b, gap));
                int child = items.Count - 1;
                while (child > 0)
                {
                    int parent = (child - 1) / 2;
                    if (items[parent].gap >= items[child].gap) break;
                    (items[parent], items[child]) = (items[child], items[parent]);
                    child = parent;
                }
            }

            public bool TryDequeue(out double a, out double b)
            {
                if (items.Count == 0)
                {
                    a = b = 0.0;
                    return false;
                }
                a = items[0].a;
                b = items[0].b;
                items[0] = items[items.Count - 1];
                items.RemoveAt(items.Count - 1);
                int parent = 0;
                while (true)
                {
                    int left = 2 * parent + 1, right = left + 1, largest = parent;
                    if (left < items.Count && items[left].gap > items[largest].gap) largest = left;
                    if (right < items.Count && items[right].gap > items[largest].gap) largest = right;
                    if (largest == parent) break;
                    (items[parent], items[largest]) = (items[largest], items[parent]);
                    parent = largest;
                }
                return true;
            }
        }
    }
}
