// SPDX-License-Identifier: MIT

using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using CADability;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Reflection;

namespace CADability.Avalonia
{
    /// <summary>
    /// Avalonia counterpart of CADability.Forms.NET8.ToolBars.
    /// Builds the standard toolbars from menu ids and routes clicks through an
    /// <see cref="ICommandHandler"/>.
    ///
    /// Two kinds of buttons, mirroring the WinForms version:
    ///  - simple commands  → a <see cref="ToggleButton"/> that executes immediately.
    ///  - popup commands    → a <see cref="SplitButton"/>. The first click (or a click on
    ///    the drop-down arrow) opens the sub-menu. Once a sub-command has been chosen, the
    ///    button shows that command's icon and a click executes it directly; the arrow still
    ///    re-opens the sub-menu to pick a different one.
    ///
    /// Phase 1: fixed layout (a wrapping host panel). User-rearrangeable toolbars and
    /// position persistence (via Settings.GlobalSettings) are a later phase.
    /// </summary>
    public class ToolBars
    {
        // Two selectable toolbar icon sizes: the compact default and a ~1.4× larger
        // variant for high-resolution displays. Persisted in Settings.GlobalSettings.
        private const int SmallIconSize = 16;
        private const int LargeIconSize = 22; // ≈ 16 * 1.4
        private const int MenuIconSize = 16;
        private const string IconSizeKey = "ToolbarIconSize";

        // Current toolbar icon edge length; switched via LargeIcons.
        private int iconSize = SmallIconSize;

        private readonly ICommandHandler commandHandler;
        private static readonly Assembly IconAssembly = typeof(ToolBars).Assembly;

        // One updater per button; called from UpdateCommandState (driven by an idle timer).
        private readonly List<Action> stateUpdaters = new();

        // ── Phase 2 layout state ────────────────────────────────────────────────
        // The panel that hosts the toolbars; current display order; hidden set.
        private Panel? host;
        private readonly Dictionary<string, Control> bars = new();
        private List<string> order = new(StandardOrder);
        private readonly HashSet<string> hidden = new();

        // Key under which order + visibility are persisted in Settings.GlobalSettings.
        private const string LayoutKey = "ToolbarLayout";

        /// <summary>Raised whenever a toolbar is shown/hidden or reordered (so the View → Toolbars menu can re-sync its checks).</summary>
        public event Action? LayoutChanged;

        /// <summary>True when the larger (~1.4×) icon variant is selected.</summary>
        public bool LargeIcons
        {
            get => iconSize >= LargeIconSize;
            set
            {
                int newSize = value ? LargeIconSize : SmallIconSize;
                if (newSize == iconSize) return;
                iconSize = newSize;
                Settings.GlobalSettings.SetValue(IconSizeKey, iconSize);
                RebuildBars();
                LayoutChanged?.Invoke();
            }
        }

        public ToolBars(ICommandHandler commandHandler)
        {
            this.commandHandler = commandHandler;
        }

        /// <summary>The standard toolbars, in display order (same set as the WinForms version).</summary>
        private static readonly string[] StandardOrder =
            { "File", "Edit", "Zoom", "Object", "Snap", "Construct" };

        /// <summary>All known toolbar names, in their canonical (default) order.</summary>
        public IReadOnlyList<string> ToolBarNames => StandardOrder;

        /// <summary>Whether the named toolbar is currently visible.</summary>
        public bool IsVisible(string name) => !hidden.Contains(name);

        /// <summary>Localized display name for a toolbar (StringTable id "Toolbar.Name.&lt;name&gt;").</summary>
        public static string DisplayName(string name) => StringTable.GetString("Toolbar.Name." + name);

        /// <summary>Builds all standard toolbars and adds them to <paramref name="host"/>.</summary>
        public void AddStandardToolBars(Panel host)
        {
            this.host = host;
            iconSize = Settings.GlobalSettings.GetIntValue(IconSizeKey, SmallIconSize);
            // Build each toolbar once; Rebuild() arranges them according to order/hidden.
            bars.Clear();
            foreach (var name in StandardOrder)
                bars[name] = BuildToolBar(name);

            LoadLayout();
            Rebuild();
        }

        /// <summary>Recreates all buttons at the current icon size and re-arranges the host.</summary>
        private void RebuildBars()
        {
            stateUpdaters.Clear();
            bars.Clear();
            foreach (var name in StandardOrder)
                bars[name] = BuildToolBar(name);
            Rebuild();
        }

        // ── Layout: order + visibility ──────────────────────────────────────────

        /// <summary>Re-populates the host panel from <see cref="order"/>, applying visibility.</summary>
        private void Rebuild()
        {
            if (host == null) return;
            host.Children.Clear();
            foreach (var name in order)
            {
                if (!bars.TryGetValue(name, out var bar)) continue;
                bar.IsVisible = !hidden.Contains(name);
                host.Children.Add(bar);
            }
        }

        /// <summary>Shows or hides a toolbar, persists, and notifies listeners.</summary>
        public void SetVisible(string name, bool visible)
        {
            bool changed = visible ? hidden.Remove(name) : hidden.Add(name);
            if (!changed) return;
            if (bars.TryGetValue(name, out var bar)) bar.IsVisible = visible;
            SaveLayout();
            LayoutChanged?.Invoke();
        }

        /// <summary>Moves a toolbar one position earlier (towards the front/left).</summary>
        public void MoveForward(string name) => Move(name, -1);

        /// <summary>Moves a toolbar one position later (towards the back/right).</summary>
        public void MoveBackward(string name) => Move(name, +1);

        private void Move(string name, int delta)
        {
            int i = order.IndexOf(name);
            if (i < 0) return;
            int j = i + delta;
            if (j < 0 || j >= order.Count) return;
            (order[i], order[j]) = (order[j], order[i]);
            Rebuild();
            SaveLayout();
            LayoutChanged?.Invoke();
        }

        // ── Persistence via Settings.GlobalSettings ─────────────────────────────
        // Serialized as "Name=1;Name=0;..." preserving order and visibility.

        private void SaveLayout()
        {
            var parts = new List<string>(order.Count);
            foreach (var name in order)
                parts.Add(name + "=" + (hidden.Contains(name) ? "0" : "1"));
            Settings.GlobalSettings.SetValue(LayoutKey, string.Join(";", parts));
        }

        private void LoadLayout()
        {
            var saved = Settings.GlobalSettings.GetStringValue(LayoutKey, "");
            if (string.IsNullOrEmpty(saved)) return;

            var newOrder = new List<string>();
            hidden.Clear();
            foreach (var part in saved.Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                var kv = part.Split('=');
                var name = kv[0];
                if (!bars.ContainsKey(name) || newOrder.Contains(name)) continue; // ignore unknown/duplicate
                newOrder.Add(name);
                if (kv.Length > 1 && kv[1] == "0") hidden.Add(name);
            }
            // Append any toolbars not mentioned in the saved layout (e.g. newly added ones).
            foreach (var name in StandardOrder)
                if (!newOrder.Contains(name)) newOrder.Add(name);

            order = newOrder;
        }

        /// <summary>Refreshes Enabled/Checked of every button. Call from an idle timer.</summary>
        public void UpdateCommandState()
        {
            foreach (var update in stateUpdaters)
                update();
        }

        // ── Toolbar / button construction ──────────────────────────────────────

        private Control BuildToolBar(string name)
        {
            var strip = new StackPanel { Orientation = Orientation.Horizontal };

            // Grip handle on the left: clicking it opens a small menu to hide/reorder
            // this toolbar (Ausblenden / Nach vorne / Nach hinten).
            strip.Children.Add(BuildGrip(name));

            foreach (var menuId in StandardMenuIds(name))
                strip.Children.Add(BuildButton(menuId));

            return new Border
            {
                Name = name,
                Margin = new global::Avalonia.Thickness(1),
                Padding = new global::Avalonia.Thickness(1),
                BorderThickness = new global::Avalonia.Thickness(1),
                BorderBrush = new SolidColorBrush(Color.FromArgb(0x30, 0x80, 0x80, 0x80)),
                Child = strip
            };
        }

        private Control BuildGrip(string name)
        {
            var grip = new Button
            {
                Content = new Border
                {
                    Width = 4,
                    Background = new SolidColorBrush(Color.FromArgb(0x80, 0x80, 0x80, 0x80)),
                    CornerRadius = new global::Avalonia.CornerRadius(2)
                },
                Padding = new global::Avalonia.Thickness(2, 0),
                Margin = new global::Avalonia.Thickness(0, 0, 2, 0),
                MinWidth = 0,
                MinHeight = 0,
                VerticalAlignment = VerticalAlignment.Stretch,
                VerticalContentAlignment = VerticalAlignment.Stretch
            };
            ToolTip.SetTip(grip, DisplayName(name));

            var hide = new MenuItem { Header = StringTable.GetString("Toolbar.Hide") };
            hide.Click += (s, e) => SetVisible(name, false);

            var forward = new MenuItem { Header = StringTable.GetString("Toolbar.MoveForward") };
            forward.Click += (s, e) => MoveForward(name);

            var backward = new MenuItem { Header = StringTable.GetString("Toolbar.MoveBackward") };
            backward.Click += (s, e) => MoveBackward(name);

            // Use the Items collection (not ItemsSource): the Avalonia browser backend
            // only realises the first element when a MenuFlyout's ItemsSource is a list
            // of already-constructed controls.
            var gripFlyout = new MenuFlyout();
            gripFlyout.Items.Add(hide);
            gripFlyout.Items.Add(forward);
            gripFlyout.Items.Add(backward);
            grip.Flyout = gripFlyout;
            return grip;
        }

        private Control BuildButton(string menuId)
            => MenuResource.IsPopup(menuId) ? BuildSplitButton(menuId) : BuildSimpleButton(menuId);

        private Control BuildSimpleButton(string menuId)
        {
            var btn = new ToggleButton { Content = MakeContent(menuId) };
            btn.Classes.Add("toolbar"); // sizing handled by App.axaml styles
            ToolTip.SetTip(btn, StringTable.GetString(menuId));

            btn.Click += (s, e) =>
            {
                commandHandler.OnCommand(menuId);
                // The handler owns the checked state; re-sync immediately so the toggle
                // does not visually "stick" on non-checkable commands.
                UpdateOne(btn, menuId);
            };

            stateUpdaters.Add(() => UpdateOne(btn, menuId));
            return btn;
        }

        private Control BuildSplitButton(string menuId)
        {
            var image = new Image { Width = iconSize, Height = iconSize };
            var bmp = LoadIcon(menuId, iconSize);
            if (bmp != null) image.Source = bmp;

            var split = new SplitButton
            {
                Content = bmp != null ? image : (object)StringTable.GetString(menuId)
            };
            split.Classes.Add("toolbar"); // sizing handled by App.axaml styles
            ToolTip.SetTip(split, StringTable.GetString(menuId));

            string? lastSubId = null;

            // Build the drop-down sub-menu from the popup definition.
            var items = new List<Control>();
            var defs = MenuResource.LoadMenuDefinition(menuId, false, commandHandler);
            foreach (var def in defs)
                AddFlyoutItem(items, def, id =>
                {
                    lastSubId = id;
                    var subBmp = LoadIcon(id, iconSize);
                    if (subBmp != null)
                    {
                        image.Source = subBmp;
                        split.Content = image;
                    }
                    ToolTip.SetTip(split, StringTable.GetString(id));
                });

            // Items collection, not ItemsSource (browser backend only shows the first
            // item when ItemsSource holds constructed controls).
            var flyout = new MenuFlyout();
            foreach (var c in items) flyout.Items.Add(c);
            split.Flyout = flyout;

            split.Click += (s, e) =>
            {
                if (string.IsNullOrEmpty(lastSubId))
                    flyout.ShowAt(split);          // first use: let the user pick a method
                else
                    commandHandler.OnCommand(lastSubId); // later: run the remembered method
            };

            stateUpdaters.Add(() =>
            {
                var cs = new CommandState();
                if (commandHandler.OnUpdateCommand(menuId, cs))
                    split.IsEnabled = cs.Enabled;
            });
            return split;
        }

        private void AddFlyoutItem(IList<Control> target, MenuWithHandler def, Action<string> onLeafSelected)
        {
            if (def.Text == "-" || def.ID == "SEPARATOR")
            {
                target.Add(new Separator());
                return;
            }

            var item = new MenuItem { Header = def.Text ?? def.ID };
            var bmp = LoadIcon(def.ID, MenuIconSize);
            if (bmp != null)
                item.Icon = new Image { Source = bmp, Width = MenuIconSize, Height = MenuIconSize };

            if (def.SubMenus is { Length: > 0 })
            {
                var subItems = new List<Control>();
                foreach (var sub in def.SubMenus)
                    AddFlyoutItem(subItems, sub, onLeafSelected);
                foreach (var c in subItems) item.Items.Add(c);
            }
            else
            {
                item.Click += (s, e) =>
                {
                    (def.Target ?? commandHandler)?.OnCommand(def.ID);
                    onLeafSelected(def.ID);
                };
            }

            target.Add(item);
        }

        // ── Helpers ─────────────────────────────────────────────────────────────

        private object MakeContent(string menuId)
        {
            var bmp = LoadIcon(menuId, iconSize);
            return bmp != null
                ? new Image { Source = bmp, Width = iconSize, Height = iconSize }
                : (object)StringTable.GetString(menuId);
        }

        private static Bitmap? LoadIcon(string menuId, int size)
            => SvgBitmapHelper.CreateBitmapFromEmbeddedSvg(menuId, size, IconAssembly);

        private void UpdateOne(ToggleButton btn, string menuId)
        {
            var cs = new CommandState();
            if (commandHandler.OnUpdateCommand(menuId, cs))
            {
                btn.IsEnabled = cs.Enabled;
                btn.IsChecked = cs.Checked;
            }
        }

        // ── Standard toolbar definitions (identical to the WinForms version) ─────

        private static string[] StandardMenuIds(string name) => name switch
        {
            "File" => new[]
            {
                "MenuId.File.New", "MenuId.File.Open", "MenuId.File.Save", "MenuId.File.Print"
            },
            "Edit" => new[]
            {
                "MenuId.Edit.Cut", "MenuId.Edit.Copy", "MenuId.Edit.Paste",
                "MenuId.Edit.Undo", "MenuId.Edit.Redo"
            },
            "Zoom" => new[]
            {
                "MenuId.Zoom.Total", "MenuId.Zoom.Detail", "MenuId.Zoom.DetailPlus",
                "MenuId.Zoom.DetailMinus", "MenuId.Repaint", "MenuId.ViewPoint",
                "MenuId.ViewFixPoint", "MenuId.Scroll", "MenuId.Zoom", "MenuId.ZAxisUp",
                "MenuId.Projection.Direction"
            },
            "Construct" => new[]
            {
                "MenuId.Select", "MenuId.Constr.Point", "MenuId.Constr.Line",
                "MenuId.Constr.Rect", "MenuId.Constr.Circle", "MenuId.Constr.Arc",
                "MenuId.Constr.Ellipse", "MenuId.Constr.Ellipsearc", "MenuId.Constr.Polyline",
                "MenuId.Constr.BSpline.Points", "MenuId.Constr.3DObjects", "MenuId.Constr.Text",
                "MenuId.Constr.Hatch", "MenuId.Constr.Picture", "MenuId.Constr.Dimension",
                "MenuId.Constr.Face", "MenuId.Tools"
            },
            "Snap" => new[]
            {
                "MenuId.Activate.Grid", "MenuId.Activate.Ortho", "MenuId.Snap.ObjectSnapPoint",
                "MenuId.Snap.ObjectPoint", "MenuId.Snap.DropPoint", "MenuId.Snap.ObjectCenter",
                "MenuId.Snap.TangentPoint", "MenuId.Snap.Intersections", "MenuId.Snap.Surface"
            },
            "Object" => new[]
            {
                "MenuId.Object.Move", "MenuId.Object.Rotate", "MenuId.Object.Scale",
                "MenuId.Object.Reflect", "MenuId.Object.Snap"
            },
            _ => Array.Empty<string>()
        };
    }
}
