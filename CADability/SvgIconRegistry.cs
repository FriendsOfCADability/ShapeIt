using System;
using System.Collections.Generic;

namespace CADability.UserInterface
{
    /// <summary>
    /// SVG icons supplied at runtime, by menu id: commands that are not known when the application is built, like
    /// the templates of ShapeIt, bring their icon as SVG text. The SvgBitmapHelpers of the front ends look here
    /// before they look for an embedded "Icons/&lt;menu id&gt;.svg", so a toolbar button, a menu item and a property
    /// entry (see <see cref="IIconPropertyEntry"/>) with that id show the icon. Used on the UI thread only.
    /// </summary>
    public static class SvgIconRegistry
    {
        private static readonly Dictionary<string, string> icons = new Dictionary<string, string>();

        /// <summary>Raised with the menu id whose icon was set, replaced or removed: rendered bitmaps of it are stale.</summary>
        public static event Action<string> IconChanged;

        /// <summary>Sets the SVG text of the icon of a menu id; null removes it.</summary>
        public static void Set(string menuId, string svg)
        {
            if (svg == null)
            {
                if (!icons.Remove(menuId)) return;
            }
            else
            {
                if (icons.TryGetValue(menuId, out string old) && old == svg) return;
                icons[menuId] = svg;
            }
            IconChanged?.Invoke(menuId);
        }

        /// <summary>The SVG text of the icon of a menu id, if one was set.</summary>
        public static bool TryGet(string menuId, out string svg)
        {
            if (menuId == null) { svg = null; return false; }
            return icons.TryGetValue(menuId, out svg);
        }
    }

    /// <summary>
    /// A property entry that shows an icon left of its label, in the column where entries with sub-entries show
    /// their +/- box. The icon is looked up like the icon of a toolbar button with that menu id.
    /// </summary>
    public interface IIconPropertyEntry
    {
        /// <summary>Menu id of the icon, null for none.</summary>
        string IconId { get; }
    }
}
