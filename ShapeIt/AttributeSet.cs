using CADability;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;

namespace ShapeIt
{
    /// <summary>
    /// A lightweight, reusable holder for the standard CADability object attributes (style, layer,
    /// color, line width, line pattern). It implements the same attribute interfaces an <see cref="IGeoObject"/>
    /// does, so the standard selection-property editors can bind to it directly - there is no need for a
    /// dummy IGeoObject just to carry the attributes in an action's property panel.
    /// <para>
    /// An action shows the editors via a <see cref="ConstructAction.InputContainer"/> (see
    /// <see cref="GetPropertyEntries"/>) and stamps the chosen values onto the created objects (see
    /// <see cref="ApplyTo"/>). The most recently applied set is kept in <see cref="LastUsed"/>, so a
    /// following action can start from the user's last choice.
    /// </para>
    /// </summary>
    internal class AttributeSet : IStyle, ILayer, IColorDef, ILineWidth, ILinePattern
    {
        private Style style;
        private ColorDef colorDef;
        private Layer layer;
        private LineWidth lineWidth;
        private LinePattern linePattern;

        /// <summary>
        /// Raised whenever any attribute changes (also while the user edits it in the property panel).
        /// Lets an action update its preview live, e.g. redraw in the newly chosen color.
        /// </summary>
        public event Action Changed;
        private void OnChanged() => Changed?.Invoke();

        public ColorDef ColorDef { get => colorDef; set { colorDef = value; OnChanged(); } }
        public Layer Layer { get => layer; set { layer = value; OnChanged(); } }
        public LineWidth LineWidth { get => lineWidth; set { lineWidth = value; OnChanged(); } }
        public LinePattern LinePattern { get => linePattern; set { linePattern = value; OnChanged(); } }

        // IColorDef also requires SetTopLevel (used for objects with children). This holder is a leaf,
        // so both overloads just set the color.
        void IColorDef.SetTopLevel(ColorDef newValue) => ColorDef = newValue;
        void IColorDef.SetTopLevel(ColorDef newValue, bool overwriteChildNullColor) => ColorDef = newValue;

        public Style Style
        {
            get => style;
            set
            {   // Applying a style also sets the individual attributes the style defines. This mirrors
                // Style.Apply, which we cannot call here because it expects an IGeoObject as its target.
                // The individual setters each raise Changed, so the preview picks up the style's color too.
                style = value;
                if (style != null)
                {
                    if (style.Layer != null) Layer = style.Layer;
                    if (style.ColorDef != null) ColorDef = style.ColorDef;
                    if (style.LineWidth != null) LineWidth = style.LineWidth;
                    if (style.LinePattern != null) LinePattern = style.LinePattern;
                }
                OnChanged();
            }
        }

        /// <summary>
        /// The most recently applied attribute set, so a following action can inherit the user's last choice.
        /// </summary>
        public static AttributeSet LastUsed { get; private set; } = new AttributeSet();

        /// <summary>
        /// Creates an attribute set pre-filled with the project defaults for the given kind of object,
        /// exactly like <c>Project.SetDefaults</c> would do for a fresh IGeoObject.
        /// </summary>
        public static AttributeSet FromDefaults(IFrame frame, Style.EDefaultFor preferredStyle)
        {
            AttributeSet res = new AttributeSet();
            res.SetDefaults(frame, preferredStyle);
            return res;
        }

        /// <summary>
        /// Fills still-unset attributes from the project's current/default attributes. Mirrors
        /// <c>Project.SetDefaults</c>, but binds to this holder instead of to an IGeoObject.
        /// </summary>
        public void SetDefaults(IFrame frame, Style.EDefaultFor preferredStyle)
        {
            Project project = frame.Project;
            Style defaultStyle = project.StyleList.GetDefault(preferredStyle);
            if (defaultStyle != null) Style = defaultStyle; // sets the individual attributes via the cascade above
            if (Layer == null && project.LayerList.Current != null) Layer = project.LayerList.Current;
            if (ColorDef == null) ColorDef = project.ColorList.Current;
            if (LineWidth == null) LineWidth = project.LineWidthList.Current;
            if (LinePattern == null) LinePattern = project.LinePatternList.Current;
        }

        /// <summary>
        /// Builds the attribute editors (style, layer, color, line width, line pattern) for the property panel.
        /// Feed the result into a <see cref="ConstructAction.InputContainer"/> via SetShowProperties. The editors
        /// write directly back into this holder. This reproduces <c>IGeoObjectImpl.GetAttributeProperties</c>.
        /// </summary>
        public IPropertyEntry[] GetPropertyEntries(IFrame frame)
        {
            Project project = frame.Project;
            List<IPropertyEntry> res = new List<IPropertyEntry>
            {
                new StyleSelectionProperty(this, "StyleSelection", project.StyleList),
                new LayerSelectionProperty(this, "LayerSelection", project.LayerList),
                new ColorSelectionProperty(this, "ColorSelection", project.ColorList, ColorList.StaticFlags.allowAll),
                new LineWidthSelectionProperty("LineWidth.Selection", project.LineWidthList, this, false),
                new LinePatternSelectionProperty("LinePatternSelection", project.LinePatternList, this, false),
            };
            return res.ToArray();
        }

        /// <summary>
        /// Stamps the attributes held here onto <paramref name="go"/> and remembers this set as <see cref="LastUsed"/>.
        /// The style is applied first so that the individual attributes below take precedence over the style's cascade.
        /// </summary>
        public void ApplyTo(IGeoObject go)
        {
            if (Style != null) go.Style = Style;
            if (Layer != null) go.Layer = Layer;
            if (go is IColorDef cd && ColorDef != null) cd.ColorDef = ColorDef;
            if (go is ILineWidth lw && LineWidth != null) lw.LineWidth = LineWidth;
            if (go is ILinePattern lp && LinePattern != null) lp.LinePattern = LinePattern;
            LastUsed = this;
        }
    }
}
