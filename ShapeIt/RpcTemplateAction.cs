using System;
using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.UserInterface;
using System.Collections;
using System.Collections.Generic;
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
        private readonly List<InputObject> errorTargets = new List<InputObject>();
        // the inputs of required object parameters (solid, profile): without them neither preview nor commit
        private readonly List<(string name, InputObject input)> requiredInputs = new List<(string name, InputObject input)>();

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
            for (int i = 0; i < parameterNames.Count; i++)
            {
                MCPServer.ParameterInfo parameterInfo = mcpServer.GetTemplateParameterInfo(templateName, parameterNames[i]);
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
                                    di.Optional = true;
                                    inputs.Add(di);
                                    errorTargets.Add(di);
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
                                    li.Optional = true;
                                    inputs.Add(li);
                                    errorTargets.Add(li);
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
                                    IntInput.Optional = true;
                                    inputs.Add(IntInput);
                                    errorTargets.Add(IntInput);
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
                                    AngleInput.Optional = true;
                                    inputs.Add(AngleInput);
                                    errorTargets.Add(AngleInput);
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
                            // others to be implemented!
                    }
                }

            }
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
        /// A parameter value has changed. The preview on screen, if any, stays until the next one is asked for:
        /// clearing it would take away the shape the user is comparing with. Only the error message goes, it
        /// was about the old values.
        /// </summary>
        private void ParameterChanged()
        {
            SetInputError(null);
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
            InputObject? missing = MissingRequiredInput();
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
            SetInputError(result.Error);
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

        private void SetInputError(string? message)
        {
            // The same message on every field: the assertion is about the combination of values, and there is
            // no way to tell which of them the template author meant.
            foreach (InputObject input in errorTargets) input.SetError(message);
        }

        public override void OnViewsChanged()
        {
            feedback.Detach();
            feedback.Attach(CurrentMouseView);
            base.OnViewsChanged();
        }

        public override void OnDone()
        {
            InputObject? missing = MissingRequiredInput();
            if (missing != null)
            {   // committed too early: stay in the action and ask for the missing object
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
