// SPDX-License-Identifier: MIT
// Hinweis: Skelettklasse zur Migration von MainMenu/ToolStripItem/ContextMenu -> MenuStrip/ToolStripMenuItem/ContextMenuStrip
// Passe die minimalen Schnittstellen unten an deine echten Typen (MenuWithHandler, ICommandHandler, CommandState, Imagequelle) an.

using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Security.Cryptography;
using System.Windows.Forms;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using Timer = System.Windows.Forms.Timer;
using ToolTip = System.Windows.Forms.ToolTip;

namespace CADability.Forms.NET8
{
    internal class ContextMenuWithHandler : ContextMenuStrip
    {
        ICommandHandler commandHandler;
        public string menuID;

        public ContextMenuWithHandler(ToolStripItem[] MenuItems, ICommandHandler Handler, string menuID) : base()
        {
            commandHandler = Handler;
            this.menuID = menuID;
        }

        public ContextMenuWithHandler(MenuWithHandler[] definition) : base()
        {
            this.menuID = null;
            commandHandler = null;
            ToolStripItem[] sub = new ToolStripItem[definition.Length];
            for (int i = 0; i < sub.Length; i++)
            {
                sub[i] = new MenuItemWithHandler(definition[i]);
            }
            base.Items.AddRange(sub);
            this.VisibleChanged += (s, e) => MenuItemWithHandler.HideToolTip();
        }

        private void RecurseCommandState(MenuItemWithHandler miid)
        {
            foreach (ToolStripMenuItem mi in miid.DropDownItems)
            {
                if (mi is MenuItemWithHandler submiid)
                {
                    if (commandHandler != null)
                    {
                        CommandState commandState = new CommandState();
                        commandHandler.OnUpdateCommand((submiid.Tag as MenuWithHandler).ID, commandState);
                        submiid.Enabled = commandState.Enabled;
                        submiid.Checked = commandState.Checked;
                    }
                    if (submiid.HasDropDownItems) RecurseCommandState(submiid);
                }
            }
        }
        public void UpdateCommand()
        {
            foreach (ToolStripItem mi in Items)
            {
                if (mi is not MenuItemWithHandler miid) continue;
                if (mi.Tag is MenuWithHandler menuWithHandler)
                {
                    if (menuWithHandler.Target != null)
                    {
                        CommandState commandState = new CommandState();
                        menuWithHandler.Target.OnUpdateCommand(menuWithHandler.ID, commandState);
                        miid.Enabled = commandState.Enabled;
                        miid.Checked = commandState.Checked;
                    }
                }
            }
        }
        public void SetCommandHandler(ICommandHandler hc)
        {
            foreach (ToolStripItem mi in Items)
            {
                if (mi is MenuItemWithHandler submiid)
                {
                    (submiid.Tag as MenuWithHandler).Target = hc;
                }
            }
            this.commandHandler = hc;
        }
        private void SetCommandHandler(MenuItemWithHandler miid, ICommandHandler hc)
        {
            foreach (ToolStripItem mi in miid.DropDownItems)
            {
                MenuItemWithHandler submiid = mi as MenuItemWithHandler;
                if (submiid != null)
                {
                    (submiid.Tag as MenuWithHandler).Target = hc;
                    if (submiid.HasDropDownItems) SetCommandHandler(submiid, hc);
                }
            }
        }
        public delegate void MenuItemSelectedDelegate(string menuId);
        public event MenuItemSelectedDelegate MenuItemSelectedEvent;
        public void FireMenuItemSelected(string menuId)
        {
            if (MenuItemSelectedEvent != null) MenuItemSelectedEvent(menuId);
        }

    }

    class MenuItemWithHandler : ToolStripMenuItem
    {
        private static Timer hoverTimer;
        public static ToolTip toolTip;
        private static Timer autoHideTimer;
        private static MenuItemWithHandler currentItem;
        private static string currentToolTipText;

        private bool doubleChecked; // the ToolStripItem Checked property behaves strange. Maybe "because it is a field of a marshal-by-reference class" is the problem? 
                                    // so here is a copy of this flag
        private static Shortcut ShortcutFromString(string p)
        {   // rather primitive:
            switch (p)
            {
                case "Alt0": return Shortcut.Alt0;
                case "Alt1": return Shortcut.Alt1;
                case "Alt2": return Shortcut.Alt2;
                case "Alt3": return Shortcut.Alt3;
                case "Alt4": return Shortcut.Alt4;
                case "Alt5": return Shortcut.Alt5;
                case "Alt6": return Shortcut.Alt6;
                case "Alt7": return Shortcut.Alt7;
                case "Alt8": return Shortcut.Alt8;
                case "Alt9": return Shortcut.Alt9;
                case "AltBksp": return Shortcut.AltBksp;
                case "AltDownArrow": return Shortcut.AltDownArrow;
                case "AltF1": return Shortcut.AltF1;
                case "AltF10": return Shortcut.AltF10;
                case "AltF11": return Shortcut.AltF11;
                case "AltF12": return Shortcut.AltF12;
                case "AltF2": return Shortcut.AltF2;
                case "AltF3": return Shortcut.AltF3;
                case "AltF4": return Shortcut.AltF4;
                case "AltF5": return Shortcut.AltF5;
                case "AltF6": return Shortcut.AltF6;
                case "AltF7": return Shortcut.AltF7;
                case "AltF8": return Shortcut.AltF8;
                case "AltF9": return Shortcut.AltF9;
                case "AltLeftArrow": return Shortcut.AltLeftArrow;
                case "AltRightArrow": return Shortcut.AltRightArrow;
                case "AltUpArrow": return Shortcut.AltUpArrow;
                case "Ctrl0": return Shortcut.Ctrl0;
                case "Ctrl1": return Shortcut.Ctrl1;
                case "Ctrl2": return Shortcut.Ctrl2;
                case "Ctrl3": return Shortcut.Ctrl3;
                case "Ctrl4": return Shortcut.Ctrl4;
                case "Ctrl5": return Shortcut.Ctrl5;
                case "Ctrl6": return Shortcut.Ctrl6;
                case "Ctrl7": return Shortcut.Ctrl7;
                case "Ctrl8": return Shortcut.Ctrl8;
                case "Ctrl9": return Shortcut.Ctrl9;
                case "CtrlA": return Shortcut.CtrlA;
                case "CtrlB": return Shortcut.CtrlB;
                case "CtrlC": return Shortcut.CtrlC;
                case "CtrlD": return Shortcut.CtrlD;
                case "CtrlDel": return Shortcut.CtrlDel;
                case "CtrlE": return Shortcut.CtrlE;
                case "CtrlF": return Shortcut.CtrlF;
                case "CtrlF1": return Shortcut.CtrlF1;
                case "CtrlF10": return Shortcut.CtrlF10;
                case "CtrlF11": return Shortcut.CtrlF11;
                case "CtrlF12": return Shortcut.CtrlF12;
                case "CtrlF2": return Shortcut.CtrlF2;
                case "CtrlF3": return Shortcut.CtrlF3;
                case "CtrlF4": return Shortcut.CtrlF4;
                case "CtrlF5": return Shortcut.CtrlF5;
                case "CtrlF6": return Shortcut.CtrlF6;
                case "CtrlF7": return Shortcut.CtrlF7;
                case "CtrlF8": return Shortcut.CtrlF8;
                case "CtrlF9": return Shortcut.CtrlF9;
                case "CtrlG": return Shortcut.CtrlG;
                case "CtrlH": return Shortcut.CtrlH;
                case "CtrlI": return Shortcut.CtrlI;
                case "CtrlIns": return Shortcut.CtrlIns;
                case "CtrlJ": return Shortcut.CtrlJ;
                case "CtrlK": return Shortcut.CtrlK;
                case "CtrlL": return Shortcut.CtrlL;
                case "CtrlM": return Shortcut.CtrlM;
                case "CtrlN": return Shortcut.CtrlN;
                case "CtrlO": return Shortcut.CtrlO;
                case "CtrlP": return Shortcut.CtrlP;
                case "CtrlQ": return Shortcut.CtrlQ;
                case "CtrlR": return Shortcut.CtrlR;
                case "CtrlS": return Shortcut.CtrlS;
                case "CtrlShift0": return Shortcut.CtrlShift0;
                case "CtrlShift1": return Shortcut.CtrlShift1;
                case "CtrlShift2": return Shortcut.CtrlShift2;
                case "CtrlShift3": return Shortcut.CtrlShift3;
                case "CtrlShift4": return Shortcut.CtrlShift4;
                case "CtrlShift5": return Shortcut.CtrlShift5;
                case "CtrlShift6": return Shortcut.CtrlShift6;
                case "CtrlShift7": return Shortcut.CtrlShift7;
                case "CtrlShift8": return Shortcut.CtrlShift8;
                case "CtrlShift9": return Shortcut.CtrlShift9;
                case "CtrlShiftA": return Shortcut.CtrlShiftA;
                case "CtrlShiftB": return Shortcut.CtrlShiftB;
                case "CtrlShiftC": return Shortcut.CtrlShiftC;
                case "CtrlShiftD": return Shortcut.CtrlShiftD;
                case "CtrlShiftE": return Shortcut.CtrlShiftE;
                case "CtrlShiftF": return Shortcut.CtrlShiftF;
                case "CtrlShiftF1": return Shortcut.CtrlShiftF1;
                case "CtrlShiftF10": return Shortcut.CtrlShiftF10;
                case "CtrlShiftF11": return Shortcut.CtrlShiftF11;
                case "CtrlShiftF12": return Shortcut.CtrlShiftF12;
                case "CtrlShiftF2": return Shortcut.CtrlShiftF2;
                case "CtrlShiftF3": return Shortcut.CtrlShiftF3;
                case "CtrlShiftF4": return Shortcut.CtrlShiftF4;
                case "CtrlShiftF5": return Shortcut.CtrlShiftF5;
                case "CtrlShiftF6": return Shortcut.CtrlShiftF6;
                case "CtrlShiftF7": return Shortcut.CtrlShiftF7;
                case "CtrlShiftF8": return Shortcut.CtrlShiftF8;
                case "CtrlShiftF9": return Shortcut.CtrlShiftF9;
                case "CtrlShiftG": return Shortcut.CtrlShiftG;
                case "CtrlShiftH": return Shortcut.CtrlShiftH;
                case "CtrlShiftI": return Shortcut.CtrlShiftI;
                case "CtrlShiftJ": return Shortcut.CtrlShiftJ;
                case "CtrlShiftK": return Shortcut.CtrlShiftK;
                case "CtrlShiftL": return Shortcut.CtrlShiftL;
                case "CtrlShiftM": return Shortcut.CtrlShiftM;
                case "CtrlShiftN": return Shortcut.CtrlShiftN;
                case "CtrlShiftO": return Shortcut.CtrlShiftO;
                case "CtrlShiftP": return Shortcut.CtrlShiftP;
                case "CtrlShiftQ": return Shortcut.CtrlShiftQ;
                case "CtrlShiftR": return Shortcut.CtrlShiftR;
                case "CtrlShiftS": return Shortcut.CtrlShiftS;
                case "CtrlShiftT": return Shortcut.CtrlShiftT;
                case "CtrlShiftU": return Shortcut.CtrlShiftU;
                case "CtrlShiftV": return Shortcut.CtrlShiftV;
                case "CtrlShiftW": return Shortcut.CtrlShiftW;
                case "CtrlShiftX": return Shortcut.CtrlShiftX;
                case "CtrlShiftY": return Shortcut.CtrlShiftY;
                case "CtrlShiftZ": return Shortcut.CtrlShiftZ;
                case "CtrlT": return Shortcut.CtrlT;
                case "CtrlU": return Shortcut.CtrlU;
                case "CtrlV": return Shortcut.CtrlV;
                case "CtrlW": return Shortcut.CtrlW;
                case "CtrlX": return Shortcut.CtrlX;
                case "CtrlY": return Shortcut.CtrlY;
                case "CtrlZ": return Shortcut.CtrlZ;
                case "Del": return Shortcut.Del;
                case "F1": return Shortcut.F1;
                case "F10": return Shortcut.F10;
                case "F11": return Shortcut.F11;
                case "F12": return Shortcut.F12;
                case "F2": return Shortcut.F2;
                case "F3": return Shortcut.F3;
                case "F4": return Shortcut.F4;
                case "F5": return Shortcut.F5;
                case "F6": return Shortcut.F6;
                case "F7": return Shortcut.F7;
                case "F8": return Shortcut.F8;
                case "F9": return Shortcut.F9;
                case "Ins": return Shortcut.Ins;
                case "None": return Shortcut.None;
                case "ShiftDel": return Shortcut.ShiftDel;
                case "ShiftF1": return Shortcut.ShiftF1;
                case "ShiftF10": return Shortcut.ShiftF10;
                case "ShiftF11": return Shortcut.ShiftF11;
                case "ShiftF12": return Shortcut.ShiftF12;
                case "ShiftF2": return Shortcut.ShiftF2;
                case "ShiftF3": return Shortcut.ShiftF3;
                case "ShiftF4": return Shortcut.ShiftF4;
                case "ShiftF5": return Shortcut.ShiftF5;
                case "ShiftF6": return Shortcut.ShiftF6;
                case "ShiftF7": return Shortcut.ShiftF7;
                case "ShiftF8": return Shortcut.ShiftF8;
                case "ShiftF9": return Shortcut.ShiftF9;
                case "ShiftIns": return Shortcut.ShiftIns;
                default: return Shortcut.None;
            }
        }
        static MenuItemWithHandler()
        {
            hoverTimer = new Timer();
            hoverTimer.Interval = 500;
            hoverTimer.Tick += HoverTimer_Tick;

            toolTip = new ToolTip();
            toolTip.AutoPopDelay = 10000; // kann man zusätzlich noch setzen
            toolTip.InitialDelay = 0;
            toolTip.ReshowDelay = 0;
            toolTip.ShowAlways = true;

            // Neuer Timer für max. Anzeigedauer
            autoHideTimer = new Timer();
            autoHideTimer.Interval = 5000; // nach 5s ausblenden
            autoHideTimer.Tick += AutoHideTimer_Tick;
        }
        public MenuItemWithHandler(MenuWithHandler definition) : base()
        {
            Text = definition.Text;
            Tag = definition;
            if (!string.IsNullOrEmpty(definition.Shortcut))
            {
                MenuManager.TryParseShortcut(definition.Shortcut, out Keys shortcutKeys);
                ShortcutKeys = shortcutKeys;
            }
            if (definition.SubMenus != null)
            {
                ToolStripItem[] sub = new ToolStripItem[definition.SubMenus.Length];
                for (int i = 0; i < definition.SubMenus.Length; i++)
                {
                    sub[i] = new MenuItemWithHandler(definition.SubMenus[i]);
                    if (definition.SubMenus[i].Text == "-") sub[i] = new ToolStripSeparator();
                }
                DropDownItems.AddRange(sub);
            }

        }
        protected override void OnClick(EventArgs e)
        {
            // Beim Anklicken ebenfalls Tooltip verstecken
            HideToolTip();

            hoverTimer.Stop();
            MenuWithHandler definition = Tag as MenuWithHandler;
            if (definition.Target != null) definition.Target.OnCommand(definition.ID);
        }
        private static void HoverTimer_Tick(object sender, EventArgs e)
        {
            // Wird ausgelöst, nachdem der Timer abgelaufen ist
            hoverTimer.Stop();
            if (currentItem != null && !string.IsNullOrEmpty(currentToolTipText))
            {
                // Position des Mauszeigers bestimmen
                Point mousePos = Cursor.Position;

                // Tooltip in Nähe des Mauszeigers anzeigen. 
                // Man könnte den Tooltip auch relativ zu einem Control platzieren.
                Form ownerForm = currentItem.GetOwnerForm();
                if (ownerForm != null)
                {
                    // In Form-Koordinaten umrechnen
                    Point formPos = ownerForm.PointToClient(mousePos);
                    toolTip.Show(currentToolTipText, ownerForm, formPos.X + 10, formPos.Y + 10);
                    autoHideTimer.Stop();
                    autoHideTimer.Start();
                }
            }
        }
        private Form GetOwnerForm()
        {
            return Form.ActiveForm;
        }
        protected override void OnOwnerChanged(EventArgs e)
        {
            base.OnOwnerChanged(e);
            if (Owner != null && Image == null)
            {
                var sz = Owner.ImageScalingSize;
                if (Tag is MenuWithHandler definition)
                {
                    // definition.ID -> versuche SVG zu laden!
                    int ind = definition.ImageIndex;
                    if (ind >= 10000) ind = ind - 10000 + ButtonImages.OffsetUserImages;
                    if (ind >= 0)
                    {
                        base.Image = ButtonImages.ButtonImageList.Images[ind];
                    }
                }
            }
        }
        private static void AutoHideTimer_Tick(object sender, EventArgs e)
        {
            // Nach 5s ausblenden
            HideToolTip();
        }
        internal static void HideToolTip()
        {
            Form? ownerForm = currentItem?.GetOwnerForm();
            if (ownerForm != null)
            {
                toolTip.Hide(ownerForm);
            }
            autoHideTimer.Stop();
        }
    }

    /// <summary>
    /// Adapter von CADability.MenuWithHandler[] nach .NET 8 WinForms (MenuStrip/ContextMenuStrip).
    /// Fokus: korrekte Verdrahtung von Command-Click und dynamischer CommandState-Aktualisierung
    /// beim Aufklappen. OwnerDraw/Custom-Renderer sind bewusst NICHT enthalten (kann später ergänzt werden).
    /// </summary>
    public static class MenuManager
    {
        /// <summary>
        /// Erzeugt einen MenuStrip aus MenuWithHandler-Definitionen. Füge das Ergebnis ins Form ein und
        /// setze Form.MainMenuStrip darauf.
        /// </summary>
        public static MenuStrip MakeMainMenu(MenuWithHandler[] definition)
        {
            MenuStrip res = new MenuStrip();
            ToolStripMenuItem[] items = new ToolStripMenuItem[definition.Length];
            for (int i = 0; i < definition.Length; i++)
            {
                items[i] = new MenuItemWithHandler(definition[i]);
            }
            res.Items.AddRange(items);
            res.MenuActivate += (s, e) => UpdateAll(res.Items);
            res.VisibleChanged += (s, e) => MenuItemWithHandler.HideToolTip();
            return res;

        }

        /// <summary>
        /// Erzeugt ein ContextMenuStrip aus MenuWithHandler-Definitionen.
        /// </summary>
        internal static ContextMenuWithHandler MakeContextMenu(MenuWithHandler[] definitions)
        {
            ContextMenuWithHandler cm = new ContextMenuWithHandler(definitions);

            // Vor dem Anzeigen dynamische States ziehen
            cm.Opening += (s, e) => UpdateAll(cm.Items);
            return cm;
        }


        #region --- Events / Command Binding ---
        private static void OnItemClick(object? sender, EventArgs e)
        {
            if (sender is not ToolStripMenuItem tsi) return;
            if (tsi.Tag is not MenuWithHandler def) return;

            var target = def.Target;
            if (target == null) return;

            try
            {
                target.OnCommand(def.ID);
            }
            catch (Exception ex)
            {
                // Logging/Diagnose nach Bedarf
                System.Diagnostics.Debug.WriteLine($"Menu command '{def.ID}' threw: {ex}");
            }
        }
        #endregion

        #region --- State-Aktualisierung ---
        private static void UpdateAll(ToolStripItemCollection items)
        {
            foreach (ToolStripItem it in items)
            {
                UpdateState(it);
                if (it is ToolStripMenuItem mi && mi.HasDropDownItems)
                {
                    // für bereits aufgebaute Untermenüs auch aktualisieren
                    UpdateAll(mi.DropDownItems);
                }
            }
        }

        private static void UpdateChildren(ToolStripMenuItem parent)
        {
            foreach (ToolStripItem it in parent.DropDownItems)
                UpdateState(it);
        }

        private static void UpdateState(ToolStripItem it)
        {
            if (it is ToolStripSeparator) return;

            if (it.Tag is not MenuWithHandler def)
                return;

            var state = new CommandState();
            var target = def.Target;

            try
            {
                target?.OnUpdateCommand(def.ID, state);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"OnUpdateCommand '{def.ID}' threw: {ex}");
            }

            // Sichtbarkeit: optional
            it.Visible = true;

            // Enable/Disable
            it.Enabled = state.Enabled;

            // Checked nur, wenn vorgesehen (oder du willst generisch alles erlauben)
            if (it is ToolStripMenuItem mi)
            {
                mi.Checked = state.Checked;
            }
        }
        #endregion

        #region --- Shortcut-Parsing ---
        /// <summary>
        /// Wandelt Shortcut-Strings aus der alten Welt in Keys um.
        /// Erlaubt z.B.: "CtrlO", "CtrlShiftS", "AltBksp", "F5", "CtrlF12", "A", "Ctrl+O".
        /// </summary>
        public static bool TryParseShortcut(string s, out Keys keys)
        {
            keys = Keys.None;
            if (string.IsNullOrWhiteSpace(s)) return false;

            // Normalisiere: entferne Leerzeichen und Pluszeichen
            s = s.Replace(" ", string.Empty).Replace("+", string.Empty);

            // Sonderfälle, die früher hart verdrahtet waren
            if (s.Equals("AltBksp", StringComparison.OrdinalIgnoreCase))
            {
                keys = Keys.Alt | Keys.Back;
                return true;
            }

            Keys mods = Keys.None;
            bool changed;
            do
            {
                changed = false;
                if (s.StartsWith("Ctrl", StringComparison.OrdinalIgnoreCase))
                {
                    mods |= Keys.Control;
                    s = s.Substring(4);
                    changed = true;
                }
                if (s.StartsWith("Shift", StringComparison.OrdinalIgnoreCase))
                {
                    mods |= Keys.Shift;
                    s = s.Substring(5);
                    changed = true;
                }
                if (s.StartsWith("Alt", StringComparison.OrdinalIgnoreCase))
                {
                    mods |= Keys.Alt;
                    s = s.Substring(3);
                    changed = true;
                }
            } while (changed && s.Length > 0);

            // F1..F24
            if (s.Length > 1 && (s[0] == 'F' || s[0] == 'f') && int.TryParse(s.Substring(1), out var f) && f >= 1 && f <= 24)
            {
                keys = mods | (Keys)Enum.Parse(typeof(Keys), "F" + f);
                return true;
            }

            // Versuche Enum-Parse (z.B. "Insert", "Delete", "Home", "End", "PageUp", "PageDown", "Back" ...)
            if (Enum.TryParse<Keys>(s, true, out var k))
            {
                keys = mods | k;
                return true;
            }

            // Einzelbuchstabe/Ziffer
            if (s.Length == 1)
            {
                char c = char.ToUpperInvariant(s[0]);
                // 0-9 und A-Z landen korrekt in Keys
                if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z'))
                {
                    keys = mods | (Keys)c;
                    return true;
                }
            }

            // letzte Chance: einige gebräuchliche Aliase
            if (TryAlias(s, out var alias))
            {
                keys = mods | alias;
                return true;
            }

            return false;
        }

        private static bool TryAlias(string s, out Keys k)
        {
            k = Keys.None;
            switch (s.ToLowerInvariant())
            {
                case "pgup": k = Keys.PageUp; return true;
                case "pgdn": k = Keys.PageDown; return true;
                case "esc": k = Keys.Escape; return true;
                case "del": k = Keys.Delete; return true;
                case "ins": k = Keys.Insert; return true;
                case "bksp": k = Keys.Back; return true;
            }
            return false;
        }
        #endregion

    }
}
