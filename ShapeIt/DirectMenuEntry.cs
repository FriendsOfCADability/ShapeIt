using CADability;
using CADability.Substitutes;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal interface IHandleKey
    {
        /// <summary>
        /// Handle the 
        /// </summary>
        /// <param name="keyEvent"></param>
        /// <returns></returns>
        bool HandleKeyCommand(KeyEventArgs keyEvent);
        bool SelectOnThisKeyStroke(KeyEventArgs keyEvent);
    }
    /// <summary>
    /// A simple line in the property grid of a property page. It shows the text of the resourceId which can be overwritten
    /// by setting the LabelText property. When selected or unselected <see cref="IsSelected"/> is beeing called, where you can
    /// give feedback in the current View. 
    /// </summary>
    internal class SelectEntry : SimplePropertyGroup, IHandleKey
    {
        private PropertyEntryType flags;
        /// <summary>
        /// Marks this entry as selectable and having a directMenu button and behaviour
        /// </summary>
        public override PropertyEntryType Flags
        {
            get
            {
                if (Menu != null) return flags | PropertyEntryType.ContextMenu;
                else return flags;
            }
        }
        public MenuWithHandler[] Menu { private get; set; }
        public SelectEntry(string resourceId, bool hasSubEntries = false) : base(resourceId)
        {
            flags = PropertyEntryType.Selectable | PropertyEntryType.Shortcut;
            if (hasSubEntries) flags = flags | PropertyEntryType.GroupTitle | PropertyEntryType.HasSubEntries;
        }
        /// <summary>
        /// Supply your code here to react on selection and deselection of this entry
        /// </summary>
        public virtual Func<bool, IFrame, bool> IsSelected { get; set; } = (selected, frame) => false;
        /// <summary>
        /// Handles the selection of this entry and calls <see cref="IsSelected"/>(true).
        /// </summary>
        /// <param name="previousSelected"></param>
        public override void Selected(IPropertyEntry previousSelected)
        {
            IsSelected(true, Frame);
            base.Selected(previousSelected);
        }
        /// <summary>
        /// Handles the de-selection of this entry and calls <see cref="IsSelected"/>(false).
        /// </summary>
        /// <param name="nowSelected"></param>
        public override void UnSelected(IPropertyEntry nowSelected)
        {
            IsSelected(false, Frame);
            base.UnSelected(nowSelected);
        }
        public Func<Keys, bool> TestShortcut { get; set; } = (key) => false;
        private IPropertyEntry FindParent(IPropertyEntry fromHere, IPropertyEntry findThis)
        {
            if (fromHere.Flags.HasFlag(PropertyEntryType.HasSubEntries))
            {
                for (int i = 0; i < fromHere.SubItems.Length; i++)
                {
                    if (fromHere.SubItems[i] == findThis) return fromHere;
                    IPropertyEntry found = FindParent(fromHere.SubItems[i], findThis);
                    if (found != null) return found;
                }
            }
            return null;
        }
        private string KeyCodeToString(Keys key)
        {
            key = (Keys)((int)key & 0xFFFF);
            string res = key.ToString();
            if (res == "Escape") return "Esc";
            if (res == "Delete") return "Del";
            return res;
        }
        public virtual bool HandleKeyCommand(KeyEventArgs keyEvent)
        {
            if (TestShortcut(keyEvent.KeyCode)) return true;
            if (Label.EndsWith("]]"))
            {
                string key = KeyCodeToString(keyEvent.KeyCode);
                int pos1 = Label.IndexOf("[[");
                if (pos1 > 0)
                {
                    int pos2 = Label.IndexOf("]]");
                    if (pos2 > pos1)
                    {
                        string shortCut = Label.Substring(pos1 + 2, pos2 - pos1 - 2);
                        bool shift = false, ctrl = false, alt = false;
                        for (int i = 0; i < 2; i++)
                        {
                            switch (shortCut[0])
                            {
                                case 's': shift = true; shortCut = shortCut.Remove(0, 1); break;
                                case 'c': ctrl = true; shortCut = shortCut.Remove(0, 1); break;
                                case 'a': alt = true; shortCut = shortCut.Remove(0, 1); break;
                            }
                        }
                        if (shift == keyEvent.KeyCode.HasFlag(Keys.Shift) &&
                            ctrl == keyEvent.KeyCode.HasFlag(Keys.Control) &&
                            alt == keyEvent.KeyCode.HasFlag(Keys.Alt) &&
                            shortCut.Contains(key))
                        {
                            // matches the keystroke
                            // IPropertyEntry pe = this.Parent as IPropertyEntry ; // this is unfortenately not correct implemented!!!
                            // so as a workaround we find the ModellingPropertyEntries and go down the tree
                            IPropertyEntry pe = propertyPage.FindFromHelpLink("Modelling.Properties");
                            if (pe != null) { pe = FindParent(pe, this); }
                            if (pe != null)
                            {
                                for (int i = 0; i < pe.SubItems.Length; i++)
                                {
                                    this.propertyPage.OpenSubEntries(pe.SubItems[i], pe.SubItems[i] == this);
                                }
                            }
                            this.propertyPage.SelectEntry(this);
                            if (this is DirectMenuEntry dm) dm.ExecuteMenu(this.Frame);
                            keyEvent.Handled = true;
                            return true;
                        }

                    }
                }
            }
            return false;
        }
        /// <summary>
        /// Simply check the first character of the Label Text
        /// </summary>
        /// <param name="keyEvent"></param>
        /// <returns></returns>
        public virtual bool SelectOnThisKeyStroke(KeyEventArgs keyEvent)
        {
            if (!string.IsNullOrEmpty(LabelText)
                && ((int)keyEvent.KeyData == (int)LabelText[0] || (int)keyEvent.KeyData == (int)LabelText.ToUpper()[0]))
                return true;
            else return false;
        }

        public override MenuWithHandler[] ContextMenu => Menu;
        public override void Added(IPropertyPage pp)
        {
            base.Added(pp);
        }
    }
    /// <summary>
    /// A simple line in the property grid of a property page. It shows the text of the resourceId which can be overwritten
    /// by setting the LabelText property. When selected or unselected <see cref="IsSelected"/> is beeing called, where you can
    /// give feedback in the current View. 
    /// It has a "directMenu" button attached to the line, which when pressed calls <see cref="ExecuteMenu"/>, where you 
    /// add your code. It als calls ExecuteMenu when the line is selected and enter is beeing pressed.
    /// </summary>
    internal class DirectMenuEntry : SelectEntry
    {
        /// <summary>
        /// Marks this entry as selectable and having a directMenu button and behaviour
        /// </summary>
        public override PropertyEntryType Flags => PropertyEntryType.Selectable | PropertyEntryType.DirectMenu | PropertyEntryType.Shortcut;
        public DirectMenuEntry(string resourceId, bool useMenu = true) : base(resourceId)
        {
        }
        /// <summary>
        /// Supply your code here, which is executed when the menu button is clicked or enter is pressed
        /// </summary>
        public Func<IFrame, bool> ExecuteMenu { get; set; } = (frame) => false;
        /// <summary>
        /// Supply your code here to react on selection and deselection of this entry
        /// </summary>
        public override Func<bool, IFrame, bool> IsSelected { get; set; } = (selected, frame) => false;
        /// <summary>
        /// Handles the menu button click or enter and forwards it to <see cref="ExecuteMenu"/>
        /// </summary>
        /// <param name="button"></param>
        public override void ButtonClicked(PropertyEntryButton button)
        {
            if (Flags.HasFlag(PropertyEntryType.DirectMenu) || button == PropertyEntryButton.doubleclick) ExecuteMenu(Frame);
        }
    }
}