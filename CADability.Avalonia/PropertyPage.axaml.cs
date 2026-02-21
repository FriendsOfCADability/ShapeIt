using Avalonia.Controls;
using CADability.UserInterface;
using System;
using System.Collections.Generic;

namespace CADability.Avalonia
{
    public partial class PropertyPage : UserControl, IPropertyPage
    {
        private PropertiesExplorer propertiesExplorer;
        private string TitleId { get; }
        private Dictionary<IPropertyEntry, PropertyEntry> properties;

        public event PreProcessKeyDown OnPreProcessKeyDown;
        public event SelectionChanged OnSelectionChanged;

        public PropertyPage(string titleId, int iconId, PropertiesExplorer propExplorer)
        {
            InitializeComponent();
            this.propertiesExplorer = propExplorer;
            this.TitleId = titleId;
            properties = new Dictionary<IPropertyEntry, PropertyEntry>();
        }

        IPropertyEntry IPropertyPage.Selected { get => throw new System.NotImplementedException(); set => throw new System.NotImplementedException(); }

        IFrame IPropertyPage.Frame => (propertiesExplorer as IControlCenter).Frame;

        IView IPropertyPage.ActiveView => throw new System.NotImplementedException();

        public void PreProcessKeyDown(Substitutes.KeyEventArgs e)
        {
            OnPreProcessKeyDown?.Invoke(e);
        }

        void IPropertyPage.Add(IPropertyEntry toAdd, bool showOpen)
        {
            Console.WriteLine("Label: " + toAdd.Label + " Index: " + toAdd.Index);
            PropertyEntry prop = new PropertyEntry(toAdd);
            properties[toAdd] = prop;
            panel.Children.Add(prop);
            // TODO select if showOpen
        }

        void IPropertyPage.BringToFront()
        {
            (propertiesExplorer as IControlCenter).ShowPropertyPage(this.TitleId);
        }

        void IPropertyPage.Clear()
        {
            panel.Children.Clear();
            properties.Clear();
        }

        bool IPropertyPage.ContainsEntry(IPropertyEntry entryWithTextBox)
        {
            throw new System.NotImplementedException();
        }

        IPropertyEntry IPropertyPage.FindFromHelpLink(string helpResourceID, bool searchTreeAndOpen)
        {
            throw new System.NotImplementedException();
        }

        IPropertyEntry IPropertyPage.GetCurrentSelection()
        {
            throw new System.NotImplementedException();
        }

        IFrame IPropertyPage.GetFrame() => (this as IPropertyPage).Frame;

        IPropertyEntry IPropertyPage.GetParent(IShowProperty child)
        {
            throw new System.NotImplementedException();
        }

        bool IPropertyPage.IsOnTop()
        {
            throw new System.NotImplementedException();
        }

        bool IPropertyPage.IsOpen(IShowProperty toTest)
        {
            throw new System.NotImplementedException();
        }

        void IPropertyPage.MakeVisible(IPropertyEntry toShow)
        {
            throw new System.NotImplementedException();
        }

        void IPropertyPage.OpenSubEntries(IPropertyEntry toOpenOrClose, bool open)
        {
            throw new System.NotImplementedException();
        }

        void IPropertyPage.Refresh(IPropertyEntry toRefresh)
        {
            throw new System.NotImplementedException();
        }

        void IPropertyPage.Remove(IPropertyEntry toRemove)
        {
            if (properties.ContainsKey(toRemove)) {
                PropertyEntry res = properties[toRemove];
                panel.Children.Remove(res);
            }
        }

        void IPropertyPage.SelectEntry(IPropertyEntry toSelect)
        {
            throw new System.NotImplementedException();
        }

        void IPropertyPage.StartEditLabel(IPropertyEntry ToEdit)
        {
            throw new System.NotImplementedException();
        }
    }
}
