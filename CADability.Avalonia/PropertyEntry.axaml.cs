using Avalonia.Controls;
using CADability.UserInterface;

namespace CADability.Avalonia
{
    public partial class PropertyEntry: UserControl
    {
        private IPropertyEntry prop;

        public PropertyEntry(IPropertyEntry prop)
        {
            InitializeComponent();
            this.prop = prop;
            if (prop.Flags.HasFlag(PropertyEntryType.LabelEditable)) {
                TextBox text = new TextBox {
                    Text = prop.Label,
                };
                this.panel.Children.Add(text);
            } else {
                TextBlock text = new TextBlock() {
                    Text = prop.Label,
                };
                this.panel.Children.Add(text);
            }
        }
    }
}
