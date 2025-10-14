using CADability.Actions;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
using Action = CADability.Actions.Action;

namespace CADability.UserInterface
{

    public class GeoPointHotSpot : IHotSpot
    {
        private GeoPoint position;
        private bool isSelected;
        public delegate void StartDragHotspotDelegate(GeoPointHotSpot sender);
        public event StartDragHotspotDelegate StartDragHotspotEvent;
        public GeoPointHotSpot(GeoPoint position)
        {
            this.position = position;
            isSelected = false;
        }
        public GeoPoint Position
        {
            get
            {
                return position;
            }
            set
            {
                position = value;
            }
        }

        #region IHotSpot Members

        /// <summary>
        /// Implements <see cref="CADability.IHotSpot.GetHotspotPosition ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual GeoPoint GetHotspotPosition()
        {
            return position;
        }

        bool IHotSpot.IsSelected
        {
            get
            {
                return isSelected;
            }
            set
            {
                isSelected = value;
            }
        }

        void IHotSpot.StartDrag(IFrame frame)
        {
            if (StartDragHotspotEvent != null) StartDragHotspotEvent(this);
        }

        string IHotSpot.GetInfoText(InfoLevelMode Level)
        {
            return null;
        }
        public string ResourceId
        {
            get { return null; }
        }

        MenuWithHandler[] IHotSpot.ContextMenu
        {
            get
            {
                return null;
            }
        }
        bool IHotSpot.Hidden
        {
            get { return false; }
        }

        #endregion
    }

    public class GeoPointProperty : EditableProperty<GeoPoint>, IUserData, ICommandHandler, IHotSpot
    {
        private NumberFormatInfo numberFormatInfo;
        private GeneralGeoPointAction generalGeoPointAction;
        private bool autoModifyWithMouse;


        /// <summary>
        /// Delegate definition for the <see cref="SelectionChangedEvent"/>
        /// </summary>
        /// <param name="sender">this object</param>
        /// <param name="isSelected">true if now selected, false otherwise</param>
        public delegate void SelectionChangedDelegate(GeoPointProperty sender, bool isSelected);
        /// <summary>
        /// This <see cref="SelectionChangedDelegate"/> event will be raised when this GeoPointProperty
        /// gets selected (the user clicks on the label or forwards the focus by pressing the tab key) or unselected.
        /// </summary>
        public event SelectionChangedDelegate SelectionChangedEvent;
        private IPropertyEntry[] subItems;
        public GeoPointProperty(IFrame frame, string resourceId = null) : base(resourceId, "MenuId.Point")
        {
            InitFormat(frame);
        }

        public GeoPointProperty(GetValueDelegate getValueDelegate, SetValueDelegate setValueDelegate, IFrame frame, string resourceId = null) : base(getValueDelegate, setValueDelegate, resourceId, "MenuId.Point")
        {
            InitFormat(frame);
        }

        public GeoPointProperty(object ObjectWithGeoPoint, string PropertyName, string resourceId, IFrame frame, bool autoModifyWithMouse = true)
            : base(ObjectWithGeoPoint, PropertyName, resourceId, "MenuId.Point")
        {
            InitFormat(frame);
        }

        private void InitFormat(IFrame frame)
        {
            Frame = frame;
            numberFormatInfo = (NumberFormatInfo)CultureInfo.CurrentCulture.NumberFormat.Clone();
            int decsym = Settings.GlobalSettings.GetIntValue("Formatting.Decimal", 0); // Systemeinstellung | Punkt | Komma
            if (decsym == 1) numberFormatInfo.NumberDecimalSeparator = ".";
            else if (decsym == 2) numberFormatInfo.NumberDecimalSeparator = ",";
            numberFormatInfo.NumberDecimalDigits = frame.GetIntSetting("Formatting.Coordinate.Digits", 3);
        }

        public bool DisplayZComponent { get; set; } = true;
        public bool AlwaysZComponent { get; set; }
        public bool UseLocalSystem { get; set; }

        protected double X
        {
            get
            {
                return GlobalToLocal(GetValue()).x;
            }
            set
            {
                GeoPoint p = GlobalToLocal(GetValue());
                p.x = value;
                InputFromSubEntries |= EInputFromSubEntries.x;
                SetValue(LocalToGlobal(p), true);
            }
        }
        protected double Y
        {
            get
            {
                return GlobalToLocal(GetValue()).y;
            }
            set
            {
                GeoPoint p = GlobalToLocal(GetValue());
                p.y = value;
                InputFromSubEntries |= EInputFromSubEntries.y;
                SetValue(LocalToGlobal(p), true);
            }
        }
        protected double Z
        {
            get
            {
                return GlobalToLocal(GetValue()).z;
            }
            set
            {
                GeoPoint p = GlobalToLocal(GetValue());
                p.z = value;
                InputFromSubEntries |= EInputFromSubEntries.z;
                SetValue(LocalToGlobal(p), true);
            }
        }
        public void SetGeoPoint(GeoPoint l)
        {
            SetValue(l, true);
        }
        public GeoPoint GetGeoPoint()
        {
            return GetValue();
        }
        public void GeoPointChanged()
        {
            propertyPage?.Refresh(this);
        }

        private GeoPoint LocalToGlobal(GeoPoint p)
        {
            if (UseLocalSystem)
            {
                Plane drawingPlane = Frame.ActiveView.Projection.DrawingPlane;
                return drawingPlane.ToGlobal(p);
            }
            return p;
        }
        private GeoPoint GlobalToLocal(GeoPoint p)
        {
            if (UseLocalSystem)
            {
                Plane drawingPlane = Frame.ActiveView.Projection.DrawingPlane;
                return drawingPlane.ToLocal(p);
            }
            return p;
        }
        public static event EventHandler<TextToValueEventArgs<GeoPoint>>? CustomTextToValue;
        static readonly string InputPattern =
    @"^\s*(?<x>[+-]?(?:\d+(?:[.,]\d*)?|[.,]\d+))(?:\s+(?<y>[+-]?(?:\d+(?:[.,]\d*)?|[.,]\d+))(?:\s+(?<z>[+-]?(?:\d+(?:[.,]\d*)?|[.,]\d+)))?)?\s*$";
        protected override bool TextToValue(string text, out GeoPoint val)
        {
            text = text.Trim();
            val = GeoPoint.Invalid;
            if (string.IsNullOrEmpty(text)) return false;
            var args = new TextToValueEventArgs<GeoPoint>(text, this);

            // Call all registered handlers
            CustomTextToValue?.Invoke(this, args);

            if (args.Handled)
            {
                // a handler has done the parsing
                val = args.Result;
                return true;
            }
            var m = Regex.Match(text, InputPattern);
            if (m.Success)
            {
                GeoPoint p = new GeoPoint(0.0, 0.0, 0.0);
                if (m.Groups["x"].Success) p.x = double.Parse(m.Groups["x"].Value, numberFormatInfo);
                if (m.Groups["y"].Success) p.y = double.Parse(m.Groups["y"].Value, numberFormatInfo);
                if (m.Groups["z"].Success) p.z = double.Parse(m.Groups["z"].Value, numberFormatInfo);
                if (m.Groups["x"].Success)
                {   // the text value might be in a local coordinate system, the value itself is always in the global system
                    val = LocalToGlobal(p);
                    return true;
                }
            }
            else if (Regex.Match(text, @"^@?[A-Za-z_][A-Za-z0-9_]*$").Success)
            {   // a named value
                object o = Frame.Project.GetNamedValue(text);
                if (o is GeoPoint pp)
                {
                    val = pp;
                    return true;
                }
            }
            return false;
        }
        protected override string ValueToText(GeoPoint p)
        {
            p = GlobalToLocal(p);
            if (DisplayZComponent || AlwaysZComponent)
            {
                return p.x.ToString("f", numberFormatInfo) + " " + p.y.ToString("f", numberFormatInfo) + " " + p.z.ToString("f", numberFormatInfo);
            }
            else
            {
                return p.x.ToString("f", numberFormatInfo) + " " + p.y.ToString("f", numberFormatInfo);
            }

        }
        public override void Selected(IPropertyEntry previousSelected)
        {
            if (previousSelected is GeoPointProperty gp) SelectionChangedEvent?.Invoke(gp, false);
            SelectionChangedEvent?.Invoke(this, true);
            base.Selected(previousSelected);
        }
        public override PropertyEntryType Flags => base.Flags | PropertyEntryType.HasSubEntries;
        public override IPropertyEntry[] SubItems
        {
            get
            {
                if (subItems == null)
                {
                    subItems = new IPropertyEntry[3];
                    subItems[0] = new DoubleProperty(this, "X", "GeoPoint.XValue", Frame)
                    {
                        DecimalDigits = Frame.GetIntSetting("Formatting.Coordinate.ComponentsDigits", 3),
                    };
                    subItems[1] = new DoubleProperty(this, "Y", "GeoPoint.YValue", Frame)
                    {
                        DecimalDigits = Frame.GetIntSetting("Formatting.Coordinate.ComponentsDigits", 3),
                    };
                    subItems[2] = new DoubleProperty(this, "Z", "GeoPoint.ZValue", Frame)
                    {
                        DecimalDigits = Frame.GetIntSetting("Formatting.Coordinate.ComponentsDigits", 3),
                    };
                }
                return subItems;
            }
        }

        #region ICommandHandler Members
        bool ICommandHandler.OnCommand(string MenuId)
        {
            if (FilterCommandEvent != null)
            {
                bool handled = false;
                FilterCommandEvent(this, MenuId, null, ref handled);
                if (handled) return true;
            }
            // maybe this property is a subproperty (e.g. Multipoint) then use the parent commandhandler first
            ICommandHandler ch = propertyPage.GetParent(this) as ICommandHandler;
            if (ch != null)
            {
                if (ch.OnCommand(MenuId)) return true;
            }
            switch (MenuId)
            {
                case "MenuId.Point.ModifyWithMouse":
                    if (ModifyWithMouse != null) ModifyWithMouse(this, false);
                    else if (autoModifyWithMouse)
                    {
                        generalGeoPointAction = new GeneralGeoPointAction(this);
                        Frame.SetAction(generalGeoPointAction);
                    }
                    return true;
                case "MenuId.Point.IntermediatePoint":
                    Frame.SetAction(new ConstructMidPoint(this));
                    return true;
                case "MenuId.Point.ObjectPoint":
                    Frame.SetAction(new ConstructObjectPoint(this));
                    return true;
                case "MenuId.Point.IntersectionTwoCurves":
                    Frame.SetAction(new ConstructIntersectPoint(this));
                    return true;
                case "MenuId.Point.OffsetByVector":
                    Frame.SetAction(new ConstructVectorPoint(this));
                    return true;
                case "MenuId.Point.Polar":
                    Frame.SetAction(new ConstructPolarPoint(this));
                    return true;
                case "MenuId.Point.NameVariable":
                    Frame.Project.SetNamedValue(null, GetGeoPoint());
                    return true;
                case "MenuId.Point.FormatSettings":
                    {
                        Frame.ShowPropertyDisplay("Global");
                        IPropertyPage pd = Frame.GetPropertyDisplay("Global");
                        IPropertyEntry sp = pd.FindFromHelpLink("Setting.Formatting");
                        if (sp != null)
                        {
                            pd.OpenSubEntries(sp, true);
                            sp = pd.FindFromHelpLink("Setting.Formatting.Coordinate");
                            if (sp != null)
                            {
                                pd.OpenSubEntries(sp, true);
                                pd.SelectEntry(sp);
                            }
                        }
                    }
                    return true;
                default:
                    return false;
            }
        }
        bool ICommandHandler.OnUpdateCommand(string MenuId, CommandState CommandState)
        {
            if (FilterCommandEvent != null)
            {
                bool handled = false;
                FilterCommandEvent(this, MenuId, CommandState, ref handled);
                if (handled) return true;
            }
            // maybe this property is a subproperty (e.g. Multipoint) then use the parent commandhandler first
            ICommandHandler ch = propertyPage.GetParent(this) as ICommandHandler;
            if (ch != null)
            {
                if (ch.OnUpdateCommand(MenuId, CommandState)) return true;
            }
            switch (MenuId)
            {
                case "MenuId.Point.ModifyWithMouse":
                    //CommandState.Checked = isModifyingWithMouse;
                    CommandState.Enabled = (!ReadOnly) && ((ModifyWithMouse != null) || autoModifyWithMouse);
                    return true;
                case "MenuId.Point.IntermediatePoint":
                case "MenuId.Point.ObjectPoint":
                case "MenuId.Point.IntersectionTwoCurves":
                case "MenuId.Point.OffsetByVector":
                case "MenuId.Point.Polar":
                    CommandState.Enabled = !ReadOnly;
                    return true;
                case "MenuId.Point.NameVariable":
                    return true;
            }
            return false;
        }
        void ICommandHandler.OnSelected(MenuWithHandler selectedMenuItem, bool selected) { }
        #endregion

        #region deprecated adaption to old implementation of GeoPointProperty
        [Obsolete("Parameter autoModifyWithMouse no longer supported, use GeoPointProperty(IFrame frame, string resourceId) instead")]
        public GeoPointProperty(string resourceId, IFrame frame, bool autoModifyWithMouse) : this(frame, resourceId)
        {
            this.autoModifyWithMouse = autoModifyWithMouse;
        }
        [Obsolete("use GeoPointProperty.SetValueDelegate instead")]
        public delegate void SetGeoPointDelegate(GeoPointProperty sender, GeoPoint p);
        [Obsolete("use GeoPointProperty.GetValueDelegate instead")]
        public delegate GeoPoint GetGeoPointDelegate(GeoPointProperty sender);
        [Obsolete("use delegate GeoPointProperty.OnSetValue instead")]
        public event SetGeoPointDelegate SetGeoPointEvent
        {
            add
            {
                base.OnSetValue = delegate (GeoPoint l) { value(this, l); };
            }
            remove
            {
                base.OnSetValue = null;
            }
        }
        [Obsolete("use delegate GeoPointProperty.OnGetValue instead")]
        public event GetGeoPointDelegate GetGeoPointEvent
        {
            add
            {
                base.OnGetValue = delegate ()
                {
                    return value(this);
                };
            }
            remove
            {
                base.OnGetValue = null;
            }
        }
        [Obsolete("use delegate GeoPointProperty.ModifyWithMouse instead")]
        public event ModifyWithMouseDelegate ModifyWithMouseEvent
        {
            add
            {
                ModifyWithMouse = delegate (IPropertyEntry pe, bool start)
                {
                    value(this, start);
                };
            }
            remove
            {
                ModifyWithMouse = null;
            }
        }
        [Obsolete("use delegate GeoPointProperty.LockedChanged instead")]
        public event LockedChangedDelegate LockedChangedEvent
        {
            add
            {
                LockedChanged = delegate (bool locked)
                {
                    value(locked);
                };
            }
            remove
            {
                LockedChanged = null;
            }
        }
        [Obsolete("method has no functionality, remove this call")]
        public void CheckMouseButton(bool Check) { }
        [Obsolete("has no funtionality")]
        public bool TabIsSpecialKeyEvent { get; internal set; }
        [Obsolete("use delegate GeoPointProperty.LabelTextChanged instead")]
        public event LabelChangedDelegate LabelChangedEvent
        {
            add
            {
                LabelTextChanged = value;
            }
            remove
            {
                LabelTextChanged = null;
            }
        }
        [Obsolete("has no funtionality")]
        public bool IsModifyingWithMouse { get; internal set; }
        #endregion

        #region IHotSpot implementation
        bool IHotSpot.IsSelected { get => propertyPage.Selected == this; set => propertyPage.SelectEntry(this); }
        MenuWithHandler[] IHotSpot.ContextMenu => ContextMenu; // ?? not sure
        bool IHotSpot.Hidden => false;
        GeoPoint IHotSpot.GetHotspotPosition()
        {
            return GetValue();
        }
        void IHotSpot.StartDrag(IFrame frame)
        {
            if (!ReadOnly)
            {
                (this as IConstructProperty).StartModifyWithMouse();
            }
        }
        string IHotSpot.GetInfoText(InfoLevelMode Level)
        {
            return Label;
        }
        #endregion

        public enum EInputFromSubEntries { x = 0x1, y = 0x2, z = 0x4 }
        public EInputFromSubEntries InputFromSubEntries = 0;
        internal GeoPoint GetPartiallyFixed(GeoPoint p)
        {
            return p;
        }
        internal void RefreshPartially()
        {
            propertyPage.Refresh(this);
        }

        /// <summary>
        /// Delegate definition for <see cref="ModifiedByActionEvent"/>
        /// </summary>
        /// <param name="sender">this object</param>
        public delegate void ModifiedByActionDelegate(GeoPointProperty sender);
        /// <summary>
        /// This <see cref="ModifiedByActionDelegate"/> event will be raised when the GeoPoint was modified by
        /// some <see cref="Action"/>. ( The <see cref="SetGeoPointEvent"/> will also be raised or the property will be set)
        /// </summary>
        public event ModifiedByActionDelegate ModifiedByActionEvent;
        internal void ModifiedByAction(Action action)
        {
            if (ModifiedByActionEvent != null) ModifiedByActionEvent(this);
        }
        public bool ForceAbsolute { get; internal set; }
        // the following should be removed and the caller should call SetContextMenu with itself as commandhandler
        public string ContextMenuId { get => GetContextMenuId(); set => SetContextMenu(value, this); }
        /// <summary>
        /// Delegate definition for <see cref="FilterCommandEvent"/>
        /// </summary>
        /// <param name="sender">this object</param>
        /// <param name="menuId">menu id of the selected menu entry</param>
        /// <param name="commandState">when not null, asks for the state of the menu</param>
        /// <param name="handled">set to true if handled</param>
        public delegate void FilterCommandDelegate(GeoPointProperty sender, string menuId, CommandState commandState, ref bool handled);
        /// <summary>
        /// When a context menue is selected or about to popup this event is raised to allow a
        /// consumer to process the command instead of this GeoPointProperty object itself.
        /// Provide a handler here if you want to process some or all menu commands.
        /// </summary>
        public event FilterCommandDelegate FilterCommandEvent;
    }
}


