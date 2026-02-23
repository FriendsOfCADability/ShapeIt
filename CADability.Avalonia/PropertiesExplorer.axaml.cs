using Avalonia.Controls;
using CADability.Substitutes;
using CADability.UserInterface;
using System.Collections.Generic;

namespace CADability.Avalonia
{

    public partial class PropertiesExplorer: UserControl, IControlCenter
    {
        private Dictionary<string, PropertyPage> tabPages;

        public PropertiesExplorer()
        {
            InitializeComponent();
            tabPages = new Dictionary<string, PropertyPage>();
        }

        IPropertyPage IControlCenter.AddPropertyPage(string titleId, int iconId)
        {
            PropertyPage page = new PropertyPage(titleId, iconId, this);
            tabPages[titleId] = page;
            TabItem tab = new TabItem {
                Header = StringTable.GetString(titleId + "TabPage", StringTable.Category.label), // TODO correct string
                Content = page,
            };
            tabControl.Items.Add(tab);

            return page;
        }

        public IPropertyPage ActivePropertyPage => (tabControl.SelectedItem as PropertyPage);

        public IFrame Frame { get; set; }
        public IPropertyEntry EntryWithTextBox { get; private set; }

        IPropertyPage IControlCenter.GetPropertyPage(string titleId)
        {
            PropertyPage ret;
            tabPages.TryGetValue(titleId, out ret);
            return ret;
        }

        void IControlCenter.DisplayHelp(string helpID)
        {
            throw new System.NotImplementedException();
        }

        bool IControlCenter.ShowPropertyPage(string titleId)
        {
            if (tabPages.ContainsKey(titleId)) {
                tabControl.SelectedItem = tabPages[titleId];
                return true;
            }
            return false;
        }

        bool IControlCenter.RemovePropertyPage(string titleId)
        {
            if (tabPages.ContainsKey(titleId)) {
                tabControl.Items.Remove(tabPages[titleId]);
                tabPages.Remove(titleId);
                return true;
            }
            return false;
        }

        void IControlCenter.PreProcessKeyDown(KeyEventArgs e)
        {
            switch (e.KeyData) {
                // case Keys.Tab: // TODO use ctrl to switch tabs
                // case Keys.Enter:
                //     if (EntryWithTextBox != null)
                //     {
                //         // TODO end editing of text box
                //         // EntryWithTextBox.EndEdit(false, textBox.Modified, textBox.Text);
                //     }

            }
            if (!e.SuppressKeyPress) {
                (ActivePropertyPage as PropertyPage)?.PreProcessKeyDown(e);
            }
        }

        void IControlCenter.HideEntry(string entryId, bool hide)
        {
            throw new System.NotImplementedException();
        }

        IPropertyEntry IControlCenter.FindItem(string name)
        {
            throw new System.NotImplementedException();
        }
    }

}
