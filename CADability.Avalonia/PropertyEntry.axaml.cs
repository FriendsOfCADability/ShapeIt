using Avalonia.Controls;
using Avalonia.Layout;
using GridPanel = Avalonia.Controls.Grid;
using CADability.UserInterface;

namespace CADability.Avalonia
{
    public partial class PropertyEntry: UserControl
    {
        private IPropertyPage parent;
        private bool showOpen;
        public IPropertyEntry Prop { get; private set; }

        public PropertyEntry(IPropertyEntry prop, IPropertyPage parent, bool showOpen)
        {
            InitializeComponent();
            this.Prop = prop;
            this.parent = parent;
            this.showOpen = showOpen;
            (parent as PropertyPage).AddToHash(this);
            createProp(prop);
            if (showOpen) {
                createSubEntries(prop);
            }
        }

        private void createProp(IPropertyEntry prop)
        {
            prop.Added(parent);
            prop.Parent = parent;
            TextBlock text = new TextBlock() {
                Text = prop.Label,
                [GridPanel.ColumnProperty] = 0,
            };
            this.panel.Children.Add(text);

            if (prop.Flags.HasFlag(PropertyEntryType.ValueEditable)) {
                TextBox value = new TextBox {
                    Text = prop.Label,
                    HorizontalAlignment = HorizontalAlignment.Right,
                    [GridPanel.ColumnProperty] = 1,
                };
                this.panel.Children.Add(value);
            } else {
                TextBlock value = new TextBlock() {
                    Text = prop.Value,
                    HorizontalAlignment = HorizontalAlignment.Right,
                    [GridPanel.ColumnProperty] = 1,
                };
                this.panel.Children.Add(value);
            }

        }

        private void createSubEntries(IPropertyEntry prop)
        {
            if (prop.SubItems == null) return;
            foreach (IPropertyEntry subProp in prop.SubItems) {
                PropertyEntry subEntry = new PropertyEntry(subProp, parent, subProp.IsOpen);
                this.childPanel.Children.Add(subEntry);
            }
        }

        public void Refresh()
        {
            // TODO
            // check if subproperties have been opened?
        }
    }
}
