// SPDX-License-Identifier: MIT

using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Threading;
using CADability.UserInterface;
using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Reflection;

namespace CADability.Avalonia
{
    /// <summary>
    /// Wraps Avalonia ContextMenu, routes ICommandHandler calls from MenuWithHandler definitions.
    /// Mirrors ContextMenuWithHandler from CADability.Forms.NET8.
    /// </summary>
    internal class ContextMenuWithHandler : ContextMenu
    {
        // Required so the Fluent/Simple theme finds the ContextMenu template.
        // Without this, Avalonia looks for a "ContextMenuWithHandler" style that does not exist,
        // leaving Template = null and rendering nothing.
        protected override Type StyleKeyOverride => typeof(ContextMenu);

        private ICommandHandler? commandHandler;
        public string? menuID;

        // Legacy constructor: accepts pre-built controls with a shared command handler
        public ContextMenuWithHandler(Control[] menuItems, ICommandHandler handler, string menuID) : base()
        {
            commandHandler = handler;
            this.menuID = menuID;
            foreach (var item in menuItems)
                Items.Add(item);
            Closing += (s, e) => MenuItemWithHandler.HideToolTip();
        }

        // Primary constructor: builds the item tree from platform-independent definitions
        public ContextMenuWithHandler(MenuWithHandler[] definition) : base()
        {
            menuID = null;
            commandHandler = null;
            foreach (var def in definition)
            {
                if (def.Text == "-")
                    Items.Add(new Separator());
                else
                    Items.Add(new MenuItemWithHandler(def));
            }
            Closing += (s, e) => MenuItemWithHandler.HideToolTip();
        }

        private void RecurseCommandState(MenuItemWithHandler item)
        {
            foreach (var child in item.Items)
            {
                if (child is MenuItemWithHandler subItem && subItem.Tag is MenuWithHandler def)
                {
                    if (commandHandler != null)
                    {
                        var state = new CommandState();
                        commandHandler.OnUpdateCommand(def.ID, state);
                        subItem.IsEnabled = state.Enabled;
                        subItem.IsChecked = state.Checked;
                    }
                    RecurseCommandState(subItem);
                }
            }
        }

        public void UpdateCommand()
        {
            foreach (var item in Items)
            {
                if (item is MenuItemWithHandler mi && mi.Tag is MenuWithHandler def && def.Target != null)
                {
                    var state = new CommandState();
                    def.Target.OnUpdateCommand(def.ID, state);
                    mi.IsEnabled = state.Enabled;
                    mi.IsChecked = state.Checked;
                }
            }
        }

        public void SetCommandHandler(ICommandHandler hc)
        {
            foreach (var item in Items)
            {
                if (item is MenuItemWithHandler mi && mi.Tag is MenuWithHandler def)
                    def.Target = hc;
            }
            commandHandler = hc;
        }

        private void SetCommandHandler(MenuItemWithHandler item, ICommandHandler hc)
        {
            foreach (var child in item.Items)
            {
                if (child is MenuItemWithHandler subItem && subItem.Tag is MenuWithHandler def)
                {
                    def.Target = hc;
                    SetCommandHandler(subItem, hc);
                }
            }
        }

        public delegate void MenuItemSelectedDelegate(string menuId);
        public event MenuItemSelectedDelegate? MenuItemSelectedEvent;

        public void FireMenuItemSelected(string menuId)
        {
            MenuItemSelectedEvent?.Invoke(menuId);
        }
    }

    /// <summary>
    /// Wraps Avalonia MenuItem, bridges MenuWithHandler data to Avalonia UI events.
    /// Mirrors MenuItemWithHandler from CADability.Forms.NET8.
    /// </summary>
    internal class MenuItemWithHandler : MenuItem
    {
        // Required so the Fluent/Simple theme finds the MenuItem template.
        protected override Type StyleKeyOverride => typeof(MenuItem);

        private static readonly DispatcherTimer hoverTimer;
        private static readonly DispatcherTimer autoHideTimer;
        // currentItem is kept for future tooltip extension; unused for now
        private static MenuItemWithHandler? currentItem;

        static MenuItemWithHandler()
        {
            hoverTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            hoverTimer.Tick += HoverTimer_Tick;

            autoHideTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(5000) };
            autoHideTimer.Tick += AutoHideTimer_Tick;
        }

        public MenuItemWithHandler(MenuWithHandler definition) : base()
        {
            // Avalonia uses Header instead of Text for display content
            Header = definition.Text;
            Tag = definition;

            if (!string.IsNullOrEmpty(definition.Shortcut) &&
                MenuManager.TryParseShortcut(definition.Shortcut, out var gesture))
            {
                InputGesture = gesture;
            }

            if (definition.SubMenus != null)
            {
                foreach (var sub in definition.SubMenus)
                {
                    if (sub.Text == "-")
                        Items.Add(new Separator());
                    else
                        Items.Add(new MenuItemWithHandler(sub));
                }
            }

            // Avalonia MenuItem.Click is not virtual; wire via event
            Click += OnItemClick;
        }

        private void OnItemClick(object? sender, RoutedEventArgs e)
        {
            HideToolTip();
            hoverTimer.Stop();
            if (Tag is MenuWithHandler definition)
                definition.Target?.OnCommand(definition.ID);
        }

        private static void HoverTimer_Tick(object? sender, EventArgs e)
        {
            hoverTimer.Stop();
            // Tooltip text can be shown here via ToolTip.SetTip(currentItem, text) if needed
        }

        private static void AutoHideTimer_Tick(object? sender, EventArgs e)
        {
            HideToolTip();
        }

        internal static void HideToolTip()
        {
            autoHideTimer.Stop();
            // Avalonia tooltips auto-hide; nothing further needed here
        }

        protected override void OnAttachedToVisualTree(global::Avalonia.VisualTreeAttachmentEventArgs e)
        {
            base.OnAttachedToVisualTree(e);
            // Load icon once the item enters the visual tree (replaces WinForms OnOwnerChanged)
            if (Icon == null && Tag is MenuWithHandler definition)
            {
                var bmp = SvgBitmapHelper.CreateBitmapFromEmbeddedSvg(
                    definition.ID,
                    assembly: Assembly.GetExecutingAssembly());
                if (bmp != null)
                    Icon = new Image { Source = bmp, Width = 16, Height = 16 };
            }
        }
    }

    /// <summary>
    /// Adapter from CADability.MenuWithHandler[] to Avalonia Menu/ContextMenu.
    /// Mirrors MenuManager from CADability.Forms.NET8.
    /// </summary>
    public static class MenuManager
    {
        /// <summary>
        /// Creates an Avalonia Menu (main menu bar) from MenuWithHandler definitions.
        /// Add the result to the window layout and set it as the main menu.
        /// </summary>
        public static Menu MakeMainMenu(MenuWithHandler[] definition)
        {
            var res = new Menu();
            foreach (var def in definition)
            {
                var item = new MenuItemWithHandler(def);
                // Refresh command states when a top-level dropdown opens
                // (replaces WinForms MenuStrip.MenuActivate)
                item.SubmenuOpened += (s, e) => UpdateAll(res.Items);
                res.Items.Add(item);
            }
            return res;
        }

        /// <summary>
        /// Creates an Avalonia ContextMenu from MenuWithHandler definitions.
        /// </summary>
        internal static ContextMenuWithHandler MakeContextMenu(MenuWithHandler[] definitions)
        {
            var cm = new ContextMenuWithHandler(definitions);
            // Refresh command states just before the menu becomes visible
            cm.Opening += (s, e) => UpdateAll(cm.Items);
            return cm;
        }

        #region State update

        private static void UpdateAll(ItemCollection items)
        {
            foreach (var it in items)
            {
                if (it is MenuItemWithHandler mi)
                {
                    UpdateState(mi);
                    if (mi.Items.Count > 0)
                        UpdateAll(mi.Items);
                }
            }
        }

        private static void UpdateState(MenuItemWithHandler mi)
        {
            if (mi.Tag is not MenuWithHandler def) return;
            var state = new CommandState();
            try
            {
                def.Target?.OnUpdateCommand(def.ID, state);
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"OnUpdateCommand '{def.ID}' threw: {ex}");
            }
            mi.IsEnabled = state.Enabled;
            if (state.Checked || state.Radio)
            {
                // ToggleType must be set for IsChecked to render the checkmark in Avalonia
                mi.ToggleType = MenuItemToggleType.CheckBox;
                mi.IsChecked = true;
            }
            else
            {
                mi.ToggleType = MenuItemToggleType.None;
                mi.IsChecked = false;
            }
        }

        #endregion

        #region Shortcut parsing

        /// <summary>
        /// Converts a shortcut string (e.g. "CtrlO", "CtrlShiftS", "AltBksp", "F5") to
        /// an Avalonia KeyGesture. Accepts the same format as the WinForms version of this method.
        /// </summary>
        public static bool TryParseShortcut(string s, out KeyGesture? gesture)
        {
            gesture = null;
            if (string.IsNullOrWhiteSpace(s)) return false;

            s = s.Replace(" ", "").Replace("+", "");

            if (s.Equals("AltBksp", StringComparison.OrdinalIgnoreCase))
            {
                gesture = new KeyGesture(Key.Back, KeyModifiers.Alt);
                return true;
            }

            KeyModifiers mods = KeyModifiers.None;
            bool changed;
            do
            {
                changed = false;
                if (s.StartsWith("Ctrl", StringComparison.OrdinalIgnoreCase))
                {
                    mods |= KeyModifiers.Control; s = s[4..]; changed = true;
                }
                if (s.StartsWith("Shift", StringComparison.OrdinalIgnoreCase))
                {
                    mods |= KeyModifiers.Shift; s = s[5..]; changed = true;
                }
                if (s.StartsWith("Alt", StringComparison.OrdinalIgnoreCase))
                {
                    mods |= KeyModifiers.Alt; s = s[3..]; changed = true;
                }
            } while (changed && s.Length > 0);

            // F1..F24
            if (s.Length > 1 && (s[0] == 'F' || s[0] == 'f') &&
                int.TryParse(s[1..], out var f) && f >= 1 && f <= 24)
            {
                if (Enum.TryParse<Key>("F" + f, out var fKey))
                {
                    gesture = new KeyGesture(fKey, mods);
                    return true;
                }
            }

            // Named keys: Delete, Insert, Home, End, PageUp, PageDown, Escape, Back ...
            if (Enum.TryParse<Key>(s, ignoreCase: true, out var k))
            {
                gesture = new KeyGesture(k, mods);
                return true;
            }

            // Single letter or digit
            if (s.Length == 1)
            {
                char c = char.ToUpperInvariant(s[0]);
                if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z'))
                {
                    if (Enum.TryParse<Key>(c.ToString(), out var lk))
                    {
                        gesture = new KeyGesture(lk, mods);
                        return true;
                    }
                }
            }

            if (TryAlias(s, out var alias))
            {
                gesture = new KeyGesture(alias, mods);
                return true;
            }

            return false;
        }

        private static bool TryAlias(string s, out Key k)
        {
            k = Key.None;
            switch (s.ToLowerInvariant())
            {
                case "pgup":  k = Key.PageUp;  return true;
                case "pgdn":  k = Key.PageDown; return true;
                case "esc":   k = Key.Escape;  return true;
                case "del":   k = Key.Delete;  return true;
                case "ins":   k = Key.Insert;  return true;
                case "bksp":  k = Key.Back;    return true;
            }
            return false;
        }

        #endregion
    }
}
