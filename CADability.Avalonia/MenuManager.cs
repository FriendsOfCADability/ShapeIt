using Avalonia.Controls;
using CADability.UserInterface;
using System;
using System.Windows.Input;

namespace CADability.Avalonia
{
    internal class MenuCommand : ICommand
    {
        ICommandHandler handler;

        public MenuCommand(ICommandHandler handler)
        {
            this.handler = handler;
        }

        public event EventHandler CanExecuteChanged;

        public bool CanExecute(object parameter)
        {
            return true;
        }

        public void Execute(object parameter)
        {
            handler.OnCommand((string)parameter);
        }
    }

    public static class MenuManager
    {
        public static void MakeMainMenu(MenuWithHandler[] definition, Menu mainMenu, ICommandHandler handler)
        {
            CreateMenuItems(definition, mainMenu, handler);
        }

        private static void CreateMenuItems(MenuWithHandler[] definition, ItemsControl menu, ICommandHandler handler)
        {
            if (definition is null) return;

            foreach (var menuDefinition in definition) {
                MenuItem item = new MenuItem {
                    Header = menuDefinition.Text,
                    Command = new MenuCommand(menuDefinition),
                    CommandParameter = menuDefinition.ID,
                };
                CreateMenuItems(menuDefinition.SubMenus, item, handler);
                menu.Items.Add(item);
            }
        }
    }
}
