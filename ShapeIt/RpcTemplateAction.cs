using System;
using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.UserInterface;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text.Json;
using System.Threading;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    internal class RpcTemplateAction : ConstructAction
    {
        private MCPServer mcpServer;
        private string templateName;
        Dictionary<string, object> parameterValues = new Dictionary<string, object>();
        // picked document solids of "solid" parameters marked 'replace': the template result takes their place
        private readonly Dictionary<string, Solid> solidsToReplace = new Dictionary<string, Solid>();

        // Preview: the "Preview" button asks the worker for the shape of the current values, the worker answers
        // on the UI thread, and what it answers goes into the feedback display. See TemplatePreview for the
        // threading. Null when there is no synchronization context to answer on; the preview is then computed
        // on the UI thread.
        private Feedback feedback;
        private TemplatePreview? preview;
        private TemplatePreviewResult? lastResult;
        // the inputs of the value parameters by name: they show the problems of the values, and a preset sets
        // them through the setter, which updates the field as if the user had typed the value
        private readonly Dictionary<string, InputObject> valueInputs = new Dictionary<string, InputObject>();
        private readonly Dictionary<string, Action<object>> valueSetters = new Dictionary<string, Action<object>>();
        // the rules of the template ('min'/'max' of the inputs, 'checks' of template.begin) and what they say
        // about the current values; errors of the last run of the template come on top
        private JsonElement parameterSpecs, checkSpecs;
        private List<ParameterProblem> problems = new List<ParameterProblem>();
        private string? runError;
        // the inputs of required object parameters (solid, profile, face): without them neither preview nor commit
        private readonly List<(string name, InputObject input)> requiredInputs = new List<(string name, InputObject input)>();
        // point parameters with 'onFace' and the face parameter they lie on
        private readonly List<(string point, string face)> pointsOnFaces = new List<(string point, string face)>();

        // What the worker computes right now: its progress is shown right of the button that started it, and the
        // answer to a commit goes into the model when it arrives.
        private enum Running { None, Preview, Commit }
        private Running running = Running.None;
        private Dictionary<string, object>? runningValues; // the values the running computation got
        private TemplatePreviewResult? resultToCommit;     // the worker's answer to a commit, taken by OnDone
        private ButtonInput? previewButton, commitButton;

        public RpcTemplateAction(MCPServer mcpServer, string templateName)
        {
            this.mcpServer = mcpServer;
            this.templateName = templateName;
            feedback = new Feedback();
        }

        public override string GetID()
        { return "Constr.RcpTemplate"; }

        public override void OnSetAction()
        {

            string? templateLabel = mcpServer.GetTemplateLabel(templateName);
            string? templateDescription = mcpServer.GetTemplateDescription(templateName);
            if (templateDescription != null)
                TitleId = "@" + templateLabel + "@" + templateDescription;
            else TitleId = "@" + templateLabel;
            List<string> parameterNames = mcpServer.GetTemplateParameters(templateName);
            parameterValues = new Dictionary<string, object>();
            List<object> inputs = []; // IInputObject is not accessible
            // in the language of the application: the rules give the messages, the presets their labels
            JsonElement beginParams = mcpServer.GetLocalizedTemplateBeginParams(templateName);
            List<TemplatePreset> presets = [];
            if (beginParams.ValueKind == JsonValueKind.Object)
            {
                beginParams.TryGetProperty("parameters", out parameterSpecs);
                beginParams.TryGetProperty("checks", out checkSpecs);
                if (beginParams.TryGetProperty("presets", out JsonElement presetSpecs)) presets = TemplateParameterRules.ReadPresets(presetSpecs);
            }
            // a preset list stands right above the first parameter it sets, e.g. the module series above the module
            Dictionary<string, List<TemplatePreset>> presetsAbove = new Dictionary<string, List<TemplatePreset>>();
            List<TemplatePreset> presetsLeft = new List<TemplatePreset>();
            foreach (TemplatePreset preset in presets)
            {
                string? first = parameterNames.Find(n => preset.Options.Any(o => o.Values.Any(v => v.Key == n)));
                if (first == null) presetsLeft.Add(preset);
                else
                {
                    if (!presetsAbove.TryGetValue(first, out List<TemplatePreset>? list)) presetsAbove[first] = list = new List<TemplatePreset>();
                    list.Add(preset);
                }
            }
            List<MCPServer.ParameterInfo> parameterInfos = parameterNames.ConvertAll(n => mcpServer.GetTemplateParameterInfo(templateName, n));
            // A solid parameter named by a face parameter gets its solid from the face the user picks: it has no
            // input of its own. Its 'replace' moves to the face.
            Dictionary<string, MCPServer.ParameterInfo> solidsOfFaces = new Dictionary<string, MCPServer.ParameterInfo>();
            for (int i = 0; i < parameterNames.Count; i++)
            {
                if (parameterInfos[i].kind == "face" && parameterInfos[i].solid is string solidName)
                {
                    int solidIndex = parameterNames.IndexOf(solidName);
                    if (solidIndex >= 0) solidsOfFaces[solidName] = parameterInfos[solidIndex];
                }
                if (parameterInfos[i].kind == "point3" && parameterInfos[i].onFace is string faceName) pointsOnFaces.Add((parameterNames[i], faceName));
            }
            // A value parameter has a default, so it never blocks the commit. It is optional only when the template
            // says so: the focus starts at the first input that is not optional and moves on to the next one when
            // a value is fixed (a point clicked, Enter in a field), the way it does in the other construct actions.
            for (int i = 0; i < parameterNames.Count; i++)
            {
                if (presetsAbove.TryGetValue(parameterNames[i], out List<TemplatePreset>? presetsHere))
                {
                    foreach (TemplatePreset preset in presetsHere) inputs.Add(PresetInput(preset));
                }
                MCPServer.ParameterInfo parameterInfo = parameterInfos[i];
                if (solidsOfFaces.ContainsKey(parameterNames[i])) continue;
                if (parameterInfo.kind == null)
                {
                    if (parameterInfo.defaultValue is double) parameterInfo.kind = "length";
                    if (parameterInfo.defaultValue is int) parameterInfo.kind = "integer";
                    if (parameterInfo.defaultValue is GeoPoint) parameterInfo.kind = "point3";
                    // more to come
                }
                if (parameterInfo.kind != null)
                {
                    switch (parameterInfo.kind)
                    {
                        case "number":
                            {   // a plain signed number without unit (a LengthInput would show it as a length)
                                if (parameterInfo.defaultValue is double d)
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = d;
                                    DoubleInput di = new DoubleInput("@" + parameterInfo.label + "@" + parameterInfo.description, d);
                                    di.SetDoubleEvent += v => { parameterValues[name] = v; ParameterChanged(); return true; };
                                    di.GetDoubleEvent += () => Convert.ToDouble(parameterValues[name]);
                                    di.Optional = parameterInfo.optional;
                                    inputs.Add(di);
                                    valueInputs[name] = di;
                                    valueSetters[name] = v => di.ForceValue(Convert.ToDouble(v, CultureInfo.InvariantCulture));
                                }
                            }
                            break;
                        case "length":
                            {
                                if (parameterInfo.defaultValue is double d)
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = d;
                                    LengthInput li = new LengthInput("@" + parameterInfo.label + "@" + parameterInfo.description, d);
                                    li.SetLengthEvent += l => { parameterValues[name] = l; ParameterChanged(); return true; };
                                    li.GetLengthEvent += () => Convert.ToDouble(parameterValues[name]);
                                    li.Optional = parameterInfo.optional;
                                    inputs.Add(li);
                                    valueInputs[name] = li;
                                    valueSetters[name] = v => li.ForceValue(Convert.ToDouble(v, CultureInfo.InvariantCulture));
                                }
                            }
                            break;
                        case "integer":
                            {
                                int intVal = int.MaxValue;
                                if (parameterInfo.defaultValue is int ii) intVal = ii;
                                if (parameterInfo.defaultValue is double dd) intVal = (int)dd;
                                if (intVal != int.MaxValue)
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = intVal;
                                    IntInput IntInput = new IntInput("@" + parameterInfo.label + "@" + parameterInfo.description, intVal);
                                    IntInput.SetIntEvent += l => { parameterValues[name] = l; ParameterChanged(); };
                                    IntInput.GetIntEvent += () => Convert.ToInt32(parameterValues[name]);
                                    IntInput.Optional = parameterInfo.optional;
                                    inputs.Add(IntInput);
                                    valueInputs[name] = IntInput;
                                    valueSetters[name] = v => IntInput.ForceValue((int)Math.Round(Convert.ToDouble(v, CultureInfo.InvariantCulture)));
                                }
                            }
                            break;
                        case "angle":
                            {
                                // degrees as double: the default was truncated to whole degrees before
                                double deg = double.NaN;
                                if (parameterInfo.defaultValue is int ii) deg = ii;
                                if (parameterInfo.defaultValue is double dd) deg = dd;
                                if (!double.IsNaN(deg))
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = deg;
                                    AngleInput AngleInput = new AngleInput("@" + parameterInfo.label + "@" + parameterInfo.description, Angle.Deg(deg));
                                    // Angle is normalized to [0, 360°): a default of -75° would come back as 285°, so the
                                    // template gets the signed value in (-180°, 180°]
                                    AngleInput.SetAngleEvent += l => { double v = l.Degree; parameterValues[name] = v > 180.0 ? v - 360.0 : v; ParameterChanged(); return true; };
                                    AngleInput.GetAngleEvent += () => Angle.Deg(Convert.ToDouble(parameterValues[name]));
                                    AngleInput.Optional = parameterInfo.optional;
                                    inputs.Add(AngleInput);
                                    valueInputs[name] = AngleInput;
                                    valueSetters[name] = v => AngleInput.ForceValue(Angle.Deg(Convert.ToDouble(v, CultureInfo.InvariantCulture)));
                                }
                            }
                            break;
                        case "point3":
                            {   // the user clicks a point in the model (with snapping) or types its coordinates
                                if (parameterInfo.defaultValue is GeoPoint gp)
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = gp;
                                    GeoPointInput pointInput = new GeoPointInput("@" + parameterInfo.label + "@" + parameterInfo.description, gp);
                                    if (parameterInfo.onFace is string onFace)
                                    {   // the point lies on the picked face (see PointOnPickedFace)
                                        pointInput.SetGeoPointExEvent += (p, didSnap) => { parameterValues[name] = PointOnPickedFace(onFace, p, didSnap); ParameterChanged(); return true; };
                                    }
                                    else pointInput.SetGeoPointEvent += p => { parameterValues[name] = p; ParameterChanged(); };
                                    pointInput.GetGeoPointEvent += () => (GeoPoint)parameterValues[name];
                                    pointInput.Optional = parameterInfo.optional;
                                    inputs.Add(pointInput);
                                    valueInputs[name] = pointInput;
                                    valueSetters[name] = v => pointInput.ForceValue((GeoPoint)v);
                                }
                            }
                            break;
                        case "solid":
                            {   // the user picks a solid of the model, the template works on a copy of it
                                string name = parameterNames[i]; // capture
                                bool replace = parameterInfo.replace;
                                GeoObjectInput solidInput = new GeoObjectInput("@" + parameterInfo.label + "@" + parameterInfo.description);
                                solidInput.MultipleInput = false;
                                solidInput.FacesOnly = false;
                                solidInput.Optional = parameterInfo.optional; // starts empty, so a required one must be picked
                                solidInput.MouseOverGeoObjectsEvent += (sender, geoObjects, up) =>
                                {
                                    Solid? found = null;
                                    for (int j = 0; j < geoObjects.Length; j++)
                                    {
                                        if (geoObjects[j] is Solid sld) { found = sld; break; }
                                    }
                                    if (up && found != null)
                                    {
                                        sender.SetGeoObject(new IGeoObject[] { found }, found);
                                        parameterValues[name] = found.Clone();
                                        if (replace)
                                        {   // a solid picked before is no longer replaced: show it again
                                            if (solidsToReplace.TryGetValue(name, out Solid? previous) && previous != found) feedback.Show(previous);
                                            solidsToReplace[name] = found;
                                        }
                                        ParameterChanged();
                                    }
                                    return found != null;
                                };
                                inputs.Add(solidInput);
                                if (!solidInput.Optional) requiredInputs.Add((name, solidInput));
                            }
                            break;
                        case "profile":
                            {   // the user picks a closed planar curve (a path is taken as a whole), which becomes a sketch
                                string name = parameterNames[i]; // capture
                                CurveInput profileInput = new CurveInput("@" + parameterInfo.label + "@" + parameterInfo.description);
                                profileInput.PreferPath = true;
                                profileInput.Optional = parameterInfo.optional; // starts empty, so a required one must be picked
                                profileInput.MouseOverCurvesEvent += (sender, curves, up) =>
                                {
                                    ICurve? usable = null;
                                    for (int j = 0; j < curves.Length; j++)
                                    {
                                        if (curves[j].IsClosed && curves[j].GetPlanarState() == PlanarState.Planar) { usable = curves[j]; break; }
                                    }
                                    if (up && usable != null)
                                    {
                                        MCPServer.Sketch? sketch = MCPServer.SketchFromCurves(new List<ICurve> { usable.Clone() });
                                        if (sketch == null) return false;
                                        sender.SetCurves(new ICurve[] { usable }, usable);
                                        parameterValues[name] = sketch;
                                        ParameterChanged();
                                    }
                                    return usable != null;
                                };
                                inputs.Add(profileInput);
                                if (!profileInput.Optional) requiredInputs.Add((name, profileInput));
                            }
                            break;
                        case "face":
                            {   // the user picks a face of a solid; the template gets that face of a copy of the solid,
                                // and the solid parameter named by 'solid' gets the copy
                                string name = parameterNames[i]; // capture
                                string? solidName = parameterInfo.solid != null && solidsOfFaces.ContainsKey(parameterInfo.solid) ? parameterInfo.solid : null;
                                bool replace = solidName != null && solidsOfFaces[solidName].replace;
                                GeoObjectInput faceInput = new GeoObjectInput("@" + parameterInfo.label + "@" + parameterInfo.description);
                                faceInput.MultipleInput = false;
                                faceInput.FacesOnly = true;
                                faceInput.Optional = parameterInfo.optional; // starts empty, so a required one must be picked
                                faceInput.MouseOverGeoObjectsEvent += (sender, geoObjects, up) =>
                                {
                                    Face? found = null;
                                    for (int j = 0; j < geoObjects.Length; j++)
                                    {
                                        if (geoObjects[j] is Face fc && MCPServer.OwningSolid(fc) != null) { found = fc; break; }
                                    }
                                    if (up && found != null)
                                    {
                                        Solid owner = MCPServer.OwningSolid(found)!;
                                        (Face copiedFace, Solid? copiedSolid) = MCPServer.CopyWithSolid(found);
                                        sender.SetGeoObject(new IGeoObject[] { found }, found);
                                        parameterValues[name] = copiedFace;
                                        if (solidName != null && copiedSolid != null)
                                        {
                                            parameterValues[solidName] = copiedSolid;
                                            if (replace)
                                            {   // a solid picked before is no longer replaced: show it again
                                                if (solidsToReplace.TryGetValue(solidName, out Solid? previous) && previous != owner) feedback.Show(previous);
                                                solidsToReplace[solidName] = owner;
                                            }
                                        }
                                        // a point on this face moves onto the new one
                                        foreach ((string pointName, string faceName) in pointsOnFaces)
                                        {
                                            if (faceName == name && parameterValues.TryGetValue(pointName, out object? point) && point is GeoPoint gp
                                                && valueSetters.TryGetValue(pointName, out Action<object>? set)) set(gp);
                                        }
                                        ParameterChanged();
                                    }
                                    return found != null;
                                };
                                inputs.Add(faceInput);
                                if (!faceInput.Optional) requiredInputs.Add((name, faceInput));
                            }
                            break;
                        case "enum":
                            {   // a drop-down list of the options, the template gets the value of the chosen one
                                List<TemplateEnumOption> options = parameterInfo.options ?? new List<TemplateEnumOption>();
                                if (options.Count > 0)
                                {
                                    string name = parameterNames[i]; // capture
                                    int start = Math.Max(0, options.FindIndex(o => TemplateParameterRules.SameEnumValue(o.Value, parameterInfo.defaultValue)));
                                    parameterValues[name] = options[start].Value;
                                    MultipleChoiceInput choiceInput = new MultipleChoiceInput("@" + parameterInfo.label + "@" + parameterInfo.description, options.Select(o => o.Label).ToArray(), start);
                                    choiceInput.GetChoiceEvent += () => options.FindIndex(o => TemplateParameterRules.SameEnumValue(o.Value, parameterValues[name]));
                                    choiceInput.SetChoiceEvent += index =>
                                    {
                                        if (index < 0 || index >= options.Count) return;
                                        parameterValues[name] = options[index].Value;
                                        ParameterChanged();
                                    };
                                    choiceInput.Optional = parameterInfo.optional;
                                    inputs.Add(choiceInput);
                                    valueInputs[name] = choiceInput;
                                    valueSetters[name] = v =>
                                    {
                                        int index = options.FindIndex(o => TemplateParameterRules.SameEnumValue(o.Value, v));
                                        if (index >= 0) choiceInput.ForceValue(index);
                                    };
                                }
                            }
                            break;
                            // others to be implemented!
                    }
                }

            }
            // presets of templates loaded from a file that set no declared parameter (template.begin refuses them)
            foreach (TemplatePreset preset in presetsLeft) inputs.Add(PresetInput(preset));
            // The template runs only when asked to: "Preview" shows the result, "Commit" (or the OK button of
            // the title line) puts it into the model. Fixing the last input no longer ends the action.
            previewButton = new ButtonInput("RpcTemplate.Preview");
            previewButton.ClickedEvent += _ => PreviewClicked();
            inputs.Add(previewButton);
            commitButton = new ButtonInput("RpcTemplate.Commit");
            commitButton.ClickedEvent += _ => CommitClicked();
            inputs.Add(commitButton);
            FinishOnLastInput = false;

            base.SetInput(inputs.ToArray());
            base.ShowAttributes = true;
            //base.ShowActiveObject = false;
            base.OnSetAction();
            UpdateProblems(); // the defaults could already violate a rule

            feedback.Attach(CurrentMouseView);
            // Captured here because OnSetAction runs on the UI thread; the worker posts its results back
            // through this context. Without one there is nothing to post to, and the preview is computed on
            // the UI thread instead.
            SynchronizationContext? uiContext = SynchronizationContext.Current;
            if (uiContext != null)
            {
                preview = new TemplatePreview(mcpServer, templateName, uiContext, OnPreviewResult, OnProgress);
            }
        }

        /// <summary>
        /// Where a point parameter with 'onFace' lies on its face: under the mouse, where the mouse beam hits the face
        /// (also through a face in front of it); a typed point or one snapped to an object point (e.g. the center of
        /// an edge) perpendicularly projected onto the surface of the face. As long as no face is picked, the point
        /// stays as given.
        /// </summary>
        private GeoPoint PointOnPickedFace(string faceName, GeoPoint p, SnapPointFinder.DidSnapModes didSnap)
        {
            if (!parameterValues.TryGetValue(faceName, out object? value) || value is not Face face) return p;
            if ((didSnap == SnapPointFinder.DidSnapModes.DidNotSnap || didSnap == SnapPointFinder.DidSnapModes.DidSnapToFaceSurface) && CurrentMouseView != null)
            {
                Axis beam = CurrentMouseView.Projection.PointBeam(CurrentMousePoint);
                GeoPoint[] hits = face.GetLineIntersection(beam.Location, beam.Direction);
                if (hits.Length > 0) return hits.OrderBy(h => (h - beam.Location) * beam.Direction).First(); // the one nearest to the viewer
            }
            return MCPServer.PointOnFace(face, p);
        }

        /// <summary>
        /// A parameter value has changed. The preview on screen, if any, stays until the next one is asked for:
        /// clearing it would take away the shape the user is comparing with. The error of the last run goes, it
        /// was about the old values, and the rules of the template are checked against the new ones right away.
        /// </summary>
        private void ParameterChanged()
        {
            runError = null;
            UpdateProblems();
        }

        /// <summary>
        /// Checks the current values against 'min'/'max' of the inputs and the 'checks' of the template and shows
        /// what fails at the parameters concerned. Cheap - only expressions over the values, no geometry - and
        /// done on a copy of the values, so it does not touch the workspace a running preview works in.
        /// </summary>
        private void UpdateProblems()
        {
            // while a required object is not picked, a rule using it cannot be evaluated: that is no problem yet
            problems = TemplateParameterRules.Check(parameterSpecs, checkSpecs, new Dictionary<string, object>(parameterValues),
                (label, isMinimum, limit) => StringTable.GetFormattedString(isMinimum ? "RpcTemplate.TooSmall" : "RpcTemplate.TooLarge", label, limit),
                ignoreEvaluationErrors: MissingRequiredInput() != null);
            ShowErrors();
        }

        /// <summary>
        /// Each value input shows the problems that name its parameter and those that name none, the error of the
        /// last run of the template on top: an assert.check of the body is about the combination of values, and
        /// there is no way to tell which of them the template author meant.
        /// </summary>
        private void ShowErrors()
        {
            foreach (KeyValuePair<string, InputObject> entry in valueInputs)
            {
                List<string> messages = problems.Where(p => p.Parameters.Count == 0 || p.Parameters.Contains(entry.Key)).Select(p => p.Message).ToList();
                if (runError != null) messages.Add(runError);
                entry.Value.SetError(messages.Count > 0 ? string.Join(" ", messages) : null!); // null clears the error
            }
        }

        /// <summary>
        /// The input of the first parameter a failing rule is about, null when all rules hold. Preview and commit
        /// are refused while a rule fails: the construction would fail too, only later and less clearly.
        /// </summary>
        private InputObject? FirstProblemInput()
        {
            if (problems.Count == 0) return null;
            foreach (ParameterProblem problem in problems)
            {
                foreach (string name in problem.Parameters)
                {
                    if (valueInputs.TryGetValue(name, out InputObject? input)) return input;
                }
            }
            return valueInputs.Values.FirstOrDefault();
        }

        /// <summary>
        /// The drop-down list of a preset. Choosing an option sets its values through the inputs, as if the user had
        /// typed them, so they stay editable; the list shows the option that matches the current values, or none.
        /// </summary>
        private MultipleChoiceInput PresetInput(TemplatePreset preset)
        {
            string[] labels = preset.Options.Select(o => o.Label).ToArray();
            MultipleChoiceInput input = new MultipleChoiceInput("@" + preset.Label + "@" + (preset.Description ?? ""), labels, -1);
            input.UnselectedText = "RpcTemplate.CustomValue";
            input.Optional = true;
            input.GetChoiceEvent += () => MatchingOption(preset);
            input.SetChoiceEvent += index => ApplyPreset(preset, index);
            return input;
        }

        private void ApplyPreset(TemplatePreset preset, int index)
        {
            if (index < 0 || index >= preset.Options.Count) return;
            // an option may compute a value from one it sets before, e.g. "b": "10*m"
            Dictionary<string, object> values = new Dictionary<string, object>(parameterValues);
            foreach (KeyValuePair<string, JsonElement> entry in preset.Options[index].Values)
            {
                try
                {
                    object value = TemplateParameterRules.EvaluateValue(entry.Value, values);
                    values[entry.Key] = value;
                    if (valueSetters.TryGetValue(entry.Key, out Action<object>? set)) set(value);
                }
                catch (Exception ex) when (ex is not OutOfMemoryException)
                {   // a value that does not fit its input (a point for a length) is left as it is
                }
            }
        }

        /// <summary>The index of the first option whose values are the current ones, -1 for none.</summary>
        private int MatchingOption(TemplatePreset preset)
        {
            Dictionary<string, object> values = new Dictionary<string, object>(parameterValues);
            for (int i = 0; i < preset.Options.Count; i++)
            {
                bool matches = true;
                foreach (KeyValuePair<string, JsonElement> entry in preset.Options[i].Values)
                {
                    try
                    {
                        object wanted = TemplateParameterRules.EvaluateValue(entry.Value, values);
                        if (!values.TryGetValue(entry.Key, out object? current) || !SameValue(current, wanted)) { matches = false; break; }
                    }
                    catch (Exception ex) when (ex is not OutOfMemoryException)
                    {
                        matches = false;
                        break;
                    }
                }
                if (matches) return i;
            }
            return -1;
        }

        private static bool SameValue(object current, object wanted)
        {
            if (TemplateParameterRules.TryToDouble(current, out double a) && TemplateParameterRules.TryToDouble(wanted, out double b))
                return Math.Abs(a - b) <= 1e-9 * Math.Max(1.0, Math.Abs(b)); // the input may have rounded the typed value
            if (current is GeoPoint p && wanted is GeoPoint q) return (p | q) <= 1e-9 * Math.Max(1.0, q.ToVector().Length);
            return Equals(current, wanted);
        }

        /// <summary>
        /// The input of a required parameter which has no value yet, null when all of them have one.
        /// </summary>
        private InputObject? MissingRequiredInput()
        {
            foreach ((string name, InputObject input) in requiredInputs)
            {
                if (!parameterValues.ContainsKey(name)) return input;
            }
            return null;
        }

        /// <summary>The "Preview" button: computes the template with the values as they stand now and shows
        /// the result without putting it into the model. Clicking it again while it computes cancels.</summary>
        private void PreviewClicked()
        {
            if (running == Running.Commit) return; // the commit uses the worker, the preview has to wait
            if (running == Running.Preview)
            {
                CancelRunning();
                return;
            }
            InputObject? missing = MissingRequiredInput() ?? FirstProblemInput();
            if (missing != null)
            {
                SetFocus(missing, true);
                return;
            }
            if (preview != null) Start(Running.Preview);
            else OnPreviewResult(TemplatePreview.ComputeNow(mcpServer, templateName, parameterValues));
        }

        /// <summary>The "Commit" button: puts the template result into the model and ends the action, like the
        /// OK button of the title line (see <see cref="OnDone"/>). Clicking it again while it computes cancels.</summary>
        private void CommitClicked()
        {
            if (running == Running.Commit)
            {
                CancelRunning();
                return;
            }
            FinishExplicitly();
        }

        /// <summary>
        /// Lets the worker compute the template for a commit, which happens when its answer arrives. A preview
        /// of the same values that is already running becomes the commit instead of starting over.
        /// </summary>
        private void StartCommit()
        {
            if (running == Running.Commit) return;
            if (running == Running.Preview && runningValues != null && SameValues(runningValues, parameterValues))
            {
                running = Running.Commit;
                commitButton!.Value = previewButton!.Value;
                previewButton.Value = null;
                return;
            }
            Start(Running.Commit);
        }

        private void Start(Running what)
        {
            if (running != Running.None) CancelRunning();
            running = what;
            runningValues = new Dictionary<string, object>(parameterValues);
            RunningButton()!.Value = "…"; // until the first step reports
            preview!.Request(parameterValues);
        }

        private void CancelRunning()
        {
            preview?.Cancel();
            EndRunning();
        }

        private void EndRunning()
        {
            running = Running.None;
            runningValues = null;
            if (previewButton != null) previewButton.Value = null;
            if (commitButton != null) commitButton.Value = null;
        }

        private ButtonInput? RunningButton()
        {
            switch (running)
            {
                case Running.Preview: return previewButton;
                case Running.Commit: return commitButton;
                default: return null;
            }
        }

        /// <summary>The worker's progress, on the UI thread: shown right of the button that started it.</summary>
        private void OnProgress(string text)
        {
            ButtonInput? button = RunningButton();
            if (button != null) button.Value = text;
        }

        /// <summary>
        /// The worker's answer, on the UI thread. A failure keeps the previous shape on screen and puts the
        /// message on the input fields - the user is usually mid-edit, and clearing the display on every
        /// intermediate value would flicker more than it would inform. The answer to a commit goes into the
        /// model right away; a failed commit leaves the action open.
        /// </summary>
        private void OnPreviewResult(TemplatePreviewResult result)
        {
            bool commit = running == Running.Commit;
            EndRunning();
            lastResult = result;
            runError = result.Error;
            ShowErrors();
            if (result.Error != null) return;

            feedback.Clear();
            foreach (Solid solid in result.Solids) feedback.CreatedObjects.Add(solid);
            // The solids the result will replace would cover it, typically the modified solid itself: hide them
            // while the preview is shown, like RoundEdgesAction does. Feedback.Detach shows them again.
            foreach (Solid toReplace in solidsToReplace.Values) feedback.Hide(toReplace);
            feedback.Refresh();
            if (commit)
            {
                resultToCommit = result; // the values may have been edited meanwhile: commit what was asked for
                FinishExplicitly();
            }
        }

        /// <summary>Whether the last preview is the result for the values as they stand now.</summary>
        private bool LastResultFits()
        {
            return lastResult != null && lastResult.Error == null && lastResult.Solids.Count > 0
                && SameValues(lastResult.Parameters, parameterValues);
        }

        public override void OnViewsChanged()
        {
            feedback.Detach();
            feedback.Attach(CurrentMouseView);
            base.OnViewsChanged();
        }

        public override void OnDone()
        {
            InputObject? missing = MissingRequiredInput() ?? FirstProblemInput();
            if (missing != null)
            {   // committed too early or with values the template refuses: stay in the action, the messages
                // are at the inputs
                SetFocus(missing, true);
                return;
            }
            if (resultToCommit == null && preview != null && !LastResultFits())
            {   // nothing computed for these values yet: compute in the background with a progress display,
                // the answer comes back to OnPreviewResult, which calls this again
                StartCommit();
                return;
            }
            // Hiding moved the replaced solids to a hidden layer; the result takes over their layer below, so
            // they must have their own one back first.
            foreach (Solid toReplace in solidsToReplace.Values) feedback.Show(toReplace);
            object? o = TakeResult();
            Project? project = FrameImpl.MainFrame?.Project;
            Style? style = null;
            if (project != null)
            {
                style = project.StyleList.GetDefault(Style.EDefaultFor.Solids);
            }
            List<Solid> results = new List<Solid>();
            if (o is Solid sld) results.Add(sld);
            else if (o is List<Solid> solids) results.AddRange(solids);
            if (results.Count > 0 && project != null)
            {
                using (project.Undo.UndoFrame)
                {
                    // a picked solid marked 'replace' (e.g. the body to be perforated) is replaced by the result,
                    // which takes over its owner, layer and style
                    IGeoObjectOwner? owner = null;
                    Solid? replaced = null;
                    foreach (Solid toReplace in solidsToReplace.Values)
                    {
                        if (toReplace.Owner == null) continue; // removed meanwhile
                        owner ??= toReplace.Owner;
                        replaced ??= toReplace;
                        toReplace.Owner.Remove(toReplace);
                    }
                    owner ??= project.GetActiveModel();
                    foreach (Solid result in results)
                    {
                        if (replaced != null)
                        {
                            if (replaced.Layer != null) result.Layer = replaced.Layer;
                            if (replaced.Style != null) result.Style = replaced.Style;
                            else if (style != null) result.Style = style;
                        }
                        else if (style != null) { result.Style = style; }
                        owner.Add(result);
                    }
                }
            }
            base.OnDone();
        }

        /// <summary>
        /// What to put into the model: the worker's answer to a commit, or the last preview when it fits the
        /// current values. Only without a worker (no synchronization context) does the template run here, on the
        /// UI thread - with dialogs enabled, so an invalid combination reports itself in a message box the way it
        /// does everywhere else in the application.
        /// </summary>
        private object? TakeResult()
        {
            bool fits = LastResultFits();
            TemplatePreviewResult? current = resultToCommit ?? (fits ? lastResult : null);
            preview?.Dispose(); // nothing computed from here on can still matter
            preview = null;
            if (current != null) return current.Solids;
            // The worker may still be finishing a step it cannot be pulled out of; wait for it, otherwise two
            // threads would build geometry at once.
            lock (mcpServer.GeometryLock)
            {
                return mcpServer.ExecuteTemplate(templateName, parameterValues);
            }
        }

        private static bool SameValues(Dictionary<string, object> a, Dictionary<string, object> b)
        {
            if (a.Count != b.Count) return false;
            foreach (KeyValuePair<string, object> entry in a)
            {
                if (!b.TryGetValue(entry.Key, out object? other)) return false;
                if (!Equals(entry.Value, other)) return false;
            }
            return true;
        }

        public override void OnRemoveAction()
        {
            preview?.Dispose();
            preview = null;
            feedback.Detach();
            base.OnRemoveAction();
        }
    }
}
