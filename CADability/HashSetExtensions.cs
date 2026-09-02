using System;
using System.Collections.Generic;

namespace CADability
{
    /// <summary>
    /// Extension methods that let <see cref="HashSet{T}"/> stand in for
    /// Wintellect.PowerCollections' <c>Set&lt;T&gt;</c>, which is being removed from CADability.
    /// <para>
    /// These methods exist so that call sites can switch to the BCL type without being rewritten
    /// in the same step. They are transitional: once a call site is genuinely reworked, prefer the
    /// idiomatic BCL form (<c>Count == 0</c>, <c>UnionWith</c>, <c>IntersectWith</c>, <c>ExceptWith</c>, ...).
    /// </para>
    /// <para>
    /// Two differences between the two set types are deliberately NOT hidden here, because hiding
    /// them would be worse than the compile error that exposes them:
    /// </para>
    /// <list type="bullet">
    /// <item><description>
    /// <c>Set&lt;T&gt;.Add</c> returns true when the item was ALREADY present and replaces it;
    /// <see cref="HashSet{T}.Add"/> returns true when the item was NEW and keeps the existing one.
    /// There is no shim for <c>Add</c> - every call site must be read.
    /// </description></item>
    /// <item><description>
    /// No <c>Union</c> is provided. LINQ already defines <c>Union</c> on <c>IEnumerable&lt;T&gt;</c>
    /// and CADability calls it on hash sets in several places; adding a more specific overload here
    /// would silently hijack those calls. Write <c>new HashSet&lt;T&gt;(a, a.Comparer)</c> followed by
    /// <c>UnionWith(b)</c> instead.
    /// </description></item>
    /// </list>
    /// <para>
    /// The counterpart of <c>Set&lt;T&gt;.Clone</c> already exists as
    /// <c>Extensions.Clone&lt;T&gt;(this HashSet&lt;T&gt;)</c> and is therefore not repeated here.
    /// </para>
    /// </summary>
    public static class HashSetExtensions
    {
        /// <summary>
        /// Adds all items of <paramref name="collection"/> to <paramref name="set"/>.
        /// Adding a set to itself is a no-op, as it was in PowerCollections.
        /// </summary>
        public static void AddMany<T>(this HashSet<T> set, IEnumerable<T> collection)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (collection == null) throw new ArgumentNullException(nameof(collection));
            // Guard against self-addition: enumerating a set while adding to it would throw.
            if (ReferenceEquals(collection, set)) return;
            foreach (T item in collection) set.Add(item);
        }

        /// <summary>
        /// Removes all items of <paramref name="collection"/> from <paramref name="set"/>
        /// and returns how many items were actually removed.
        /// </summary>
        public static int RemoveMany<T>(this HashSet<T> set, IEnumerable<T> collection)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (collection == null) throw new ArgumentNullException(nameof(collection));
            if (ReferenceEquals(collection, set))
            {
                // Special case, otherwise the enumeration below would throw.
                int all = set.Count;
                set.Clear();
                return all;
            }
            int count = 0;
            foreach (T item in collection)
            {
                if (set.Remove(item)) ++count;
            }
            return count;
        }

        /// <summary>
        /// True if every item of <paramref name="collection"/> is contained in <paramref name="set"/>.
        /// </summary>
        public static bool ContainsAll<T>(this HashSet<T> set, IEnumerable<T> collection)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (collection == null) throw new ArgumentNullException(nameof(collection));
            foreach (T item in collection)
            {
                if (!set.Contains(item)) return false;
            }
            return true;
        }

        /// <summary>
        /// True if the set contains no items.
        /// </summary>
        public static bool IsEmpty<T>(this HashSet<T> set)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            return set.Count == 0;
        }

        /// <summary>
        /// Returns an arbitrary item of the set, or <c>default(T)</c> if the set is empty.
        /// Does not throw on an empty set - unlike <c>Enumerable.First</c>.
        /// </summary>
        public static T GetAny<T>(this HashSet<T> set)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            foreach (T item in set) return item;
            return default(T);
        }

        /// <summary>
        /// Returns an arbitrary item of the set and removes it, or <c>default(T)</c> if the set is empty.
        /// </summary>
        public static T GetAndRemoveAny<T>(this HashSet<T> set)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            T result = set.GetAny();
            set.Remove(result);
            return result;
        }

        /// <summary>
        /// True if both sets contain the same items. Same as <see cref="HashSet{T}.SetEquals"/>,
        /// kept under the PowerCollections name.
        /// </summary>
        public static bool IsEqualTo<T>(this HashSet<T> set, HashSet<T> other)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (other == null) throw new ArgumentNullException(nameof(other));
            return set.SetEquals(other);
        }

        /// <summary>
        /// A new set with the items contained in both sets. Neither operand is modified.
        /// </summary>
        public static HashSet<T> Intersection<T>(this HashSet<T> set, HashSet<T> other)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (other == null) throw new ArgumentNullException(nameof(other));
            HashSet<T> result = new HashSet<T>(set, set.Comparer);
            result.IntersectWith(other);
            return result;
        }

        /// <summary>
        /// A new set with the items of <paramref name="set"/> that are not in <paramref name="other"/>.
        /// Neither operand is modified.
        /// </summary>
        public static HashSet<T> Difference<T>(this HashSet<T> set, HashSet<T> other)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (other == null) throw new ArgumentNullException(nameof(other));
            HashSet<T> result = new HashSet<T>(set, set.Comparer);
            result.ExceptWith(other);
            return result;
        }

        /// <summary>
        /// A new set with the items contained in exactly one of the two sets. Neither operand is modified.
        /// </summary>
        public static HashSet<T> SymmetricDifference<T>(this HashSet<T> set, HashSet<T> other)
        {
            if (set == null) throw new ArgumentNullException(nameof(set));
            if (other == null) throw new ArgumentNullException(nameof(other));
            HashSet<T> result = new HashSet<T>(set, set.Comparer);
            result.SymmetricExceptWith(other);
            return result;
        }
    }
}
