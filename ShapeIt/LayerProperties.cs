using CADability;
using CADability.Attribute;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using CheckState = CADability.Substitutes.CheckState;

namespace ShapeIt
{
    /// <summary>
    /// A property entry that is attached to the <see cref="Project"/> as a <see cref="UserData"/> object
    /// (under the name <see cref="UserDataName"/>) and is therefore shown in the project's property page.
    /// It lists all layers of the project. Each layer offers the properties "visible" and "selectable".
    ///
    /// The object is intentionally NOT serializable: it is recreated every time a project is opened
    /// (see <see cref="EnsureExists"/>), so it is never written to or read from a saved project file.
    /// The "selectable" state however IS persisted: it lives in a dedicated <see cref="Filter"/>
    /// (named <see cref="SelectableLayersFilterName"/>) inside the project's <see cref="FilterList"/>,
    /// which is part of the saved project.
    /// </summary>
    internal class LayerProperties : PropertyEntryImpl
    {
        /// <summary>
        /// Key under which this object is stored in the project's <see cref="UserData"/>.
        /// Also serves as the resource id for the label and tooltip (see StringTable*.xml).
        /// </summary>
        public const string UserDataName = "ShapeIt.LayerProperties";

        /// <summary>
        /// Name of the <see cref="Filter"/> in the project's <see cref="FilterList"/> that governs which
        /// layers may be selected. The filter accepts exactly the layers it contains (whitelist), so a
        /// layer is selectable iff it is contained in this filter.
        /// </summary>
        public const string SelectableLayersFilterName = "ShapeIt.SelectableLayers";

        private readonly Project project;
        private readonly Filter selectableFilter;
        private IPropertyEntry[] subEntries;

        private LayerProperties(Project project) : base(UserDataName)
        {
            this.project = project;
            this.selectableFilter = EnsureSelectableFilter(project);
            project.LayerList.LayerAddedEvent += OnLayerAdded;
            project.LayerList.LayerRemovedEvent += OnLayerRemoved;
        }

        /// <summary>
        /// Makes sure the provided project carries the "ShapeIt.LayerProperties" user data object.
        /// If it does not exist yet, it is created and added.
        /// </summary>
        public static void EnsureExists(Project project)
        {
            if (project == null) return;
            if (!project.UserData.ContainsData(UserDataName))
            {
                project.UserData.Add(UserDataName, new LayerProperties(project));
            }
        }

        /// <summary>
        /// Returns the project's selectable-layers filter, creating it (with all current layers, i.e.
        /// "everything selectable") if it does not exist yet.
        /// </summary>
        private static Filter EnsureSelectableFilter(Project project)
        {
            FilterList filterList = project.FilterList;
            Filter filter = filterList.FindFilter(SelectableLayersFilterName);
            if (filter == null)
            {
                filter = Filter.Construct();
                filter.Name = SelectableLayersFilterName;
                filter.IsActive = true;
                LayerList layerList = project.LayerList;
                for (int i = 0; i < layerList.Count; ++i) filter.Add(layerList[i]);
                filterList.Add(filter);
            }
            return filter;
        }

        // New layers are selectable by default; removed layers drop out of the filter.
        private void OnLayerAdded(LayerList sender, Layer added)
        {
            selectableFilter.Add(added);
            Refresh();
        }

        private void OnLayerRemoved(LayerList sender, Layer removed)
        {
            selectableFilter.Remove(removed);
            Refresh();
        }

        public override PropertyEntryType Flags =>
            PropertyEntryType.GroupTitle | PropertyEntryType.HasSubEntries | PropertyEntryType.Selectable | PropertyEntryType.ContextMenu;

        public override MenuWithHandler[] ContextMenu
        {
            get
            {
                MenuWithHandler allVisible = new MenuWithHandler("MenuId.ShapeIt.LayerProperties.AllVisible");
                allVisible.OnCommand = (menuId) =>
                {
                    ModelView modelView = FrameImpl.MainFrame?.ActiveView as ModelView;
                    if (modelView != null)
                    {
                        LayerList layerList = project.LayerList;
                        for (int i = 0; i < layerList.Count; ++i) modelView.SetLayerVisibility(layerList[i], true);
                    }
                    Refresh();
                    return true;
                };
                MenuWithHandler allSelectable = new MenuWithHandler("MenuId.ShapeIt.LayerProperties.AllSelectable");
                allSelectable.OnCommand = (menuId) =>
                {
                    LayerList layerList = project.LayerList;
                    for (int i = 0; i < layerList.Count; ++i) selectableFilter.Add(layerList[i]);
                    Refresh();
                    return true;
                };
                return new MenuWithHandler[] { allVisible, allSelectable };
            }
        }

        public override IPropertyEntry[] SubItems
        {
            get
            {
                if (subEntries == null)
                {
                    LayerList layerList = project.LayerList;
                    List<IPropertyEntry> entries = new List<IPropertyEntry>();
                    for (int i = 0; i < layerList.Count; ++i)
                    {
                        entries.Add(new SingleLayerProperty(layerList[i], selectableFilter));
                    }
                    subEntries = entries.ToArray();
                }
                return subEntries;
            }
        }

        /// <summary>
        /// Forces the list of layers to be rebuilt the next time it is displayed.
        /// </summary>
        public override void Refresh()
        {
            subEntries = null;
            base.Refresh();
        }
    }

    /// <summary>
    /// Represents a single layer inside <see cref="LayerProperties"/>. The label is the layer name,
    /// the sub entries hold the layer properties "visible" and "selectable".
    /// </summary>
    internal class SingleLayerProperty : PropertyEntryImpl
    {
        private readonly Layer layer;
        private readonly Filter selectableFilter;
        private IPropertyEntry[] subEntries;

        public SingleLayerProperty(Layer layer, Filter selectableFilter) : base("ShapeIt.LayerProperties.Entry")
        {
            this.layer = layer;
            this.selectableFilter = selectableFilter;
            LabelText = layer.Name;
        }

        public override PropertyEntryType Flags =>
            PropertyEntryType.GroupTitle | PropertyEntryType.HasSubEntries | PropertyEntryType.Selectable;

        public override IPropertyEntry[] SubItems
        {
            get
            {
                if (subEntries == null)
                {
                    CheckState visible = IsLayerVisible() ? CheckState.Checked : CheckState.Unchecked;
                    CheckProperty visibleProperty = new CheckProperty("ShapeIt.LayerProperties.Visible", visible);
                    visibleProperty.CheckStateChangedEvent += OnVisibleChanged;

                    CheckState selectable = selectableFilter.Contains(layer) ? CheckState.Checked : CheckState.Unchecked;
                    CheckProperty selectableProperty = new CheckProperty("ShapeIt.LayerProperties.Selectable", selectable);
                    selectableProperty.CheckStateChangedEvent += OnSelectableChanged;

                    subEntries = new IPropertyEntry[] { visibleProperty, selectableProperty };
                }
                return subEntries;
            }
        }

        // ShapeIt always shows exactly one view. The project does not know which ModelView displays it,
        // so we reach it through the main frame's active view.
        private static ModelView CurrentModelView => FrameImpl.MainFrame?.ActiveView as ModelView;

        private bool IsLayerVisible()
        {
            ModelView modelView = CurrentModelView;
            if (modelView == null) return true;
            return Array.IndexOf(modelView.GetVisibleLayers(), layer) >= 0;
        }

        private void OnVisibleChanged(string label, CheckState state)
        {
            CurrentModelView?.SetLayerVisibility(layer, state == CheckState.Checked);
        }

        private void OnSelectableChanged(string label, CheckState state)
        {
            if (state == CheckState.Checked) selectableFilter.Add(layer);
            else selectableFilter.Remove(layer);
        }
    }
}
