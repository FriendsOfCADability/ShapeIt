using System;
using System.Collections;
using System.Collections.Generic;
using CADability.GeoObject;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Collections.Immutable;

namespace CADability
{

    sealed class ImmutableSetComparer<T> : IEqualityComparer<ImmutableHashSet<T>>
    {
        private readonly IEqualityComparer<T> _elem;

        public ImmutableSetComparer(IEqualityComparer<T> elementComparer = null)
            => _elem = elementComparer ?? EqualityComparer<T>.Default;

        public bool Equals(ImmutableHashSet<T> x, ImmutableHashSet<T> y)
        {
            if (ReferenceEquals(x, y)) return true;
            if (x is null || y is null) return false;
            // Set-Gleichheit (inhaltlich)
            return x.SetEquals(y);
        }

        public int GetHashCode(ImmutableHashSet<T> set)
        {
            if (set is null) return 0;

            // Ordnungunabhängiger Hash: Summe der Element-Hashes (kommutativ).
            // Alternativen: XOR, oder HashCode.Aggregate (aber Summen sind stabil & simpel).
            int hash = 0;
            foreach (var item in set)
            {
                // Null-handling nur relevant, falls T nullable ist
                hash = unchecked(hash + (_elem.GetHashCode(item!) * 397));
            }
            return hash;
        }
    }

    sealed class UnorderedPairComparer<T> : IEqualityComparer<(T A, T B)>
    {
        private static readonly IEqualityComparer<T> C = EqualityComparer<T>.Default;

        public bool Equals((T A, T B) x, (T A, T B) y)
        {
            // same or exchanged order
            return (C.Equals(x.A, y.A) && C.Equals(x.B, y.B)) ||
                   (C.Equals(x.A, y.B) && C.Equals(x.B, y.A));
        }

        public int GetHashCode((T A, T B) p)
        {
            // commutativ Hash-Combination: h(A,B) == h(B,A)
            unchecked
            {
                int h1 = p.A == null ? 0 : C.GetHashCode(p.A);
                int h2 = p.B == null ? 0 : C.GetHashCode(p.B);
                return h1 ^ h2;
            }
        }
    }

    public class CombinedEnumerable<T> : IEnumerable<T>, IEnumerator<T>, IEnumerator
    {
        IEnumerable<T>[] toEnumerate;
        int currentIndex;
        IEnumerator<T> currentEnumerator;
        public CombinedEnumerable(params IEnumerable<T>[] enums)
        {
            toEnumerate = enums;
            currentIndex = 0;
            currentEnumerator = null;
        }

        T IEnumerator<T>.Current
        {
            get
            {
                return currentEnumerator.Current;
            }
        }

        object IEnumerator.Current
        {
            get
            {
                return currentEnumerator.Current;
            }
        }

        void IDisposable.Dispose()
        {
            currentIndex = -1;
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return this;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return this;
        }

        bool IEnumerator.MoveNext()
        {
            if (currentEnumerator == null) currentEnumerator = toEnumerate[0].GetEnumerator();
            while (!currentEnumerator.MoveNext())
            {
                ++currentIndex;
                if (currentIndex < toEnumerate.Length)
                {
                    currentEnumerator = toEnumerate[currentIndex].GetEnumerator();
                }
                else
                {
                    return false;
                }
            }
            return true;
        }

        void IEnumerator.Reset()
        {
            currentIndex = 0;
            currentEnumerator = null;
        }
        //public static IEnumerable<T> operator +(IEnumerable<T> t1, IEnumerable<T> t2)
        //{
        //    return new CombinedEnumerable(t1, t2);
        //}
    }

    public class LookedUpEnumerable<T> : IEnumerable<T>, IEnumerator<T>, IEnumerator
    {
        IEnumerable<T> toEnumerate;
        IEnumerator<T> currentEnumerator;
        Dictionary<T, T> lookUp;
        public LookedUpEnumerable(IEnumerable<T> enumerable, Dictionary<T, T> lookUp)
        {
            toEnumerate = enumerable;
            currentEnumerator = null;
            this.lookUp = lookUp;
            currentEnumerator = enumerable.GetEnumerator();
        }

        T IEnumerator<T>.Current
        {
            get
            {
                if (lookUp.TryGetValue(currentEnumerator.Current, out T val)) return val;
                else return currentEnumerator.Current;
            }
        }

        object IEnumerator.Current
        {
            get
            {
                if (lookUp.TryGetValue(currentEnumerator.Current, out T val)) return val;
                else return currentEnumerator.Current;
            }
        }

        void IDisposable.Dispose()
        {
            currentEnumerator.Dispose();
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return this;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return this;
        }

        bool IEnumerator.MoveNext()
        {
            return currentEnumerator.MoveNext();
        }

        void IEnumerator.Reset()
        {
            currentEnumerator = null;
        }
    }

    internal class SerializableHashSetString : HashSet<string>, IJsonSerialize
    {
        public SerializableHashSetString() : base() { }
        public SerializableHashSetString(IEnumerable<string> strings) : base(strings) { }
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Strings", this.ToArray());
        }

        public void SetObjectData(IJsonReadData data)
        {
            string[] strings = data.GetProperty<string[]>("Strings");
            for (int i = 0; i < strings.Length; i++)
            {
                Add(strings[i]);
            }
        }
    }
    public static partial class Extensions
    {
        public static IEnumerable<T> Combine<T>(params IEnumerable<T>[] enumerators)
        {
            return new CombinedEnumerable<T>(enumerators);
        }
        public static IEnumerable<T> LookUp<T>(IEnumerable<T> enumertor, Dictionary<T, T> lookUp)
        {
            return new LookedUpEnumerable<T>(enumertor, lookUp);
        }
        public static Matrix RowVector(params GeoVector[] v)
        {
            double[,] A = new double[3, v.Length];
            for (int i = 0; i < v.Length; ++i)
            {
                A[0, i] = v[i].x;
                A[1, i] = v[i].y;
                A[2, i] = v[i].z;
            }
            return DenseMatrix.OfArray(A);
        }
        public static Matrix ColumnVector(params GeoVector[] v1)
        {
            double[,] A = new double[v1.Length, 3];
            for (int i = 0; i < v1.Length; ++i)
            {
                A[i, 0] = v1[i].x;
                A[i, 1] = v1[i].y;
                A[i, 2] = v1[i].z;
            }
            return DenseMatrix.OfArray(A);
        }
        public static bool IsValid(this Matrix matrix)
        {
            return matrix.RowCount > 0 && !double.IsNaN(matrix[0, 0]) && !double.IsInfinity(matrix[0, 0]);
        }
        public static bool IsValid(this Vector v)
        {
            return v.Count > 0 && !double.IsNaN(v[0]) && !double.IsInfinity(v[0]);
        }

        internal static T[] ToArray<T>(this IEnumerable<T> e)
        {
            if (e is T[] t) return t;
            if (e is List<T> l) return l.ToArray();
            List<T> r = new List<T>();
            foreach (T item in e) r.Add(item);
            return r.ToArray();
        }

        internal static TSource MinBy<TSource, TKey>(this IEnumerable<TSource> source, Func<TSource, TKey> selector) where TKey : IComparable<TKey>
        {
            bool first = true;
            TSource minElement = default;
            TKey minValue = default;

            foreach (var item in source)
            {
                var value = selector(item);
                if (first || value.CompareTo(minValue) < 0)
                {
                    first = false;
                    minValue = value;
                    minElement = item;
                }
            }
            return minElement;
        }
        public static TSource MinByWithDefault<TSource, TKey>(this IEnumerable<TSource> source, TSource def, Func<TSource, TKey> selector) where TKey : IComparable<TKey>
        {
            bool first = true;
            TSource minElement = def;
            TKey minValue = default;

            foreach (var item in source)
            {
                var value = selector(item);
                if (first || value.CompareTo(minValue) < 0)
                {
                    first = false;
                    minValue = value;
                    minElement = item;
                }
            }
            return minElement;
        }
        public static (TSource Element, TKey Value) MinByElementAndValue<TSource, TKey>(this IEnumerable<TSource> source, Func<TSource, TKey> selector) where TKey : IComparable<TKey>
        {
            bool first = true;
            TSource minElement = default;
            TKey minValue = default;

            foreach (var item in source)
            {
                var value = selector(item);
                if (first || value.CompareTo(minValue) < 0)
                {
                    first = false;
                    minValue = value;
                    minElement = item;
                }
            }
            return (minElement, minValue);
        }

        public static void AddIfNotNull<T>(this ICollection<T> list, T item)
        {
            if (item != null)
            {
                list.Add(item);
            }
        }

        /// <summary>
        /// IEnumerable&lt;T&gt; to HashSet&lt;T&gt; 
        /// </summary>
        public static HashSet<T> ToHashSet<T>(this IEnumerable<T> source)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            return new HashSet<T>(source);
        }

        /// <summary>
        /// IEnumerable&lt;T&gt; to HashSet&lt;T&gt; with Comparer.
        /// </summary>
        public static HashSet<T> ToHashSet<T>(this IEnumerable<T> source, IEqualityComparer<T> comparer)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            return new HashSet<T>(source, comparer);
        }
        public static HashSet<T> Clone<T>(this HashSet<T> source)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            return new HashSet<T>(source, source.Comparer);
        }
        /// <summary>
        /// Remove duplicates from the list.
        /// </summary>
        /// <param name="sortedList"></param>
        /// <param name="tolerance"></param>
        public static void RemoveDuplicatesWithTolerance(this List<double> sortedList, double tolerance)
        {
            if (sortedList.Count < 2) return;

            int writeIndex = 1;
            double lastValue = sortedList[0];

            for (int i = 1; i < sortedList.Count; i++)
            {
                double current = sortedList[i];
                if (Math.Abs(current - lastValue) > tolerance)
                {
                    sortedList[writeIndex++] = current;
                    lastValue = current;
                }
            }

            // remove overhang
            sortedList.RemoveRange(writeIndex, sortedList.Count - writeIndex);
        }
        public static GeoObjectList Show(this IEnumerable<Edge> edges)
        {
            GeoObjectList res = new GeoObjectList();
            foreach (Edge e in edges)
            {
                if (e.Curve3D is IGeoObject geoObject) { res.Add(geoObject); }
            }
            return res;
        }

    }
}
