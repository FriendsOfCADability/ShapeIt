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
            foreach (var menuDefinition in definition) {
                MenuItem item = new MenuItem {
                    Header = menuDefinition.Text,
                    Command = new MenuCommand(handler),
                    CommandParameter = menuDefinition.ID,
                };
                foreach (var subMenuItem in menuDefinition.SubMenus) {
                    if (subMenuItem.ID == "SEPARATOR") {
                        item.Items.Add(new Separator());
                    } else {
                        MenuItem subItem = new MenuItem {
                            Header = subMenuItem.Text,
                            Command = new MenuCommand(handler),
                            CommandParameter = subMenuItem.ID,
                        };
                        item.Items.Add(subItem);
                    }
                }
                mainMenu.Items.Add(item);
                // TODO sub menu items recursive?
            }
        }
    }
}
