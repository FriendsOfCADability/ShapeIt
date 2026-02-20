using Avalonia.Controls;
using CADability.UserInterface;

namespace CADability.Avalonia
{
    public partial class PropertyPage : UserControl, IPropertyPage
    {
        private PropertiesExplorer propertiesExplorer;

        public PropertyPage(string titleId, int iconId, PropertiesExplorer propExplorer)
        {
            this.propertiesExplorer = propExplorer;
        }

        IPropertyEntry IPropertyPage.Selected { get => throw new System.NotImplementedException(); set => throw new System.NotImplementedException(); }

        IFrame IPropertyPage.Frame => (propertiesExplorer as IControlCenter).Frame;

        IView IPropertyPage.ActiveView => throw new System.NotImplementedException();

        event PreProcessKeyDown IPropertyPage.OnPreProcessKeyDown
        {
            add
            {
                throw new System.NotImplementedException();
            }

            remove
            {
                throw new System.NotImplementedException();
            }
        }

        event SelectionChanged IPropertyPage.OnSelectionChanged
        {
            add
            {
                throw new System.NotImplementedException();
            }

            remove
            {
                throw new System.NotImplementedException();
            }
        }

        void IPropertyPage.Add(IPropertyEntry toAdd, bool showOpen)
        {
            // throw new System.NotImplementedException();
            // TODO
        }

        void IPropertyPage.BringToFront()
        {
            // throw new System.NotImplementedException();
            // TODO
        }

        void IPropertyPage.Clear()
        {
            // throw new System.NotImplementedException();
            // TODO
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
            // throw new System.NotImplementedException();
            // TODO
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
