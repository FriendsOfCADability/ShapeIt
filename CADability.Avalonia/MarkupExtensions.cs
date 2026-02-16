using CADability.UserInterface;
using System;

namespace CADability.Avalonia
{
    public class StringTableExtension
    {
        public string Key { get; set; } = "";

        public string ProvideValue(IServiceProvider serviceProvider)
        {
            return StringTable.GetString(Key) ?? Key;
        }
    }
}
