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

        // Live preview: every parameter change asks the worker for a new shape, the worker answers on the UI
        // thread, and what it answers goes into the feedback display. See TemplatePreview for the threading.
        // Null when there is no synchronization context to answer on; the action then behaves as it did
        // before, computing only on OK.
        private Feedback feedback;
        private TemplatePreview? preview;
        private TemplatePreviewResult? lastResult;
        private readonly List<InputObject> errorTargets = new List<InputObject>();

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
                        case "length":
                            {
                                if (parameterInfo.defaultValue is double d)
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = d;
                                    LengthInput li = new LengthInput("@" + parameterInfo.label + "@" + parameterInfo.description, d);
                                    li.SetLengthEvent += l => { parameterValues[name] = l; RequestPreview(); return true; };
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
                                    IntInput.SetIntEvent += l => { parameterValues[name] = l; RequestPreview(); };
                                    IntInput.GetIntEvent += () => Convert.ToInt32(parameterValues[name]);
                                    IntInput.Optional = true;
                                    inputs.Add(IntInput);
                                    errorTargets.Add(IntInput);
                                }
                            }
                            break;
                        case "angle":
                            {
                                int intVal = int.MaxValue;
                                if (parameterInfo.defaultValue is int ii) intVal = ii;
                                if (parameterInfo.defaultValue is double dd) intVal = (int)dd;
                                if (intVal != int.MaxValue)
                                {
                                    string name = parameterNames[i]; // capture
                                    parameterValues[name] = intVal;
                                    AngleInput AngleInput = new AngleInput("@" + parameterInfo.label + "@" + parameterInfo.description, intVal);
                                    AngleInput.SetAngleEvent += l => { parameterValues[name] = l.Degree; RequestPreview(); return true; };
                                    AngleInput.GetAngleEvent += () => Angle.Deg(Convert.ToDouble(parameterValues[name]));
                                    AngleInput.Optional = true;
                                    inputs.Add(AngleInput);
                                    errorTargets.Add(AngleInput);
                                }
                            }
                            break;
                            // others to be implemented!
                    }
                }

            }
            base.SetInput(inputs.ToArray());
            base.ShowAttributes = true;
            //base.ShowActiveObject = false;
            base.OnSetAction();

            feedback.Attach(CurrentMouseView);
            // Captured here because OnSetAction runs on the UI thread; the worker posts its results back
            // through this context. Without one there is nothing to post to, so the preview stays off and the
            // action works as it did before - computing once, on OK.
            SynchronizationContext? uiContext = SynchronizationContext.Current;
            if (uiContext != null)
            {
                //preview = new TemplatePreview(mcpServer, templateName, uiContext, OnPreviewResult);
                //RequestPreview(); // show the default values right away instead of an empty screen
            }
        }

        /// <summary>Asks for a preview of the values as they stand now. Cheap: the worker sorts out
        /// superseding, cancelling and serializing.</summary>
        private void RequestPreview()
        {
            preview?.Request(parameterValues);
        }

        /// <summary>
        /// The worker's answer, on the UI thread. A failure keeps the previous shape on screen and puts the
        /// message on the input fields - the user is usually mid-edit, and clearing the display on every
        /// intermediate value would flicker more than it would inform.
        /// </summary>
        private void OnPreviewResult(TemplatePreviewResult result)
        {
            lastResult = result;
            SetInputError(result.Error);
            if (result.Error != null) return;

            feedback.Clear();
            foreach (Solid solid in result.Solids) feedback.CreatedObjects.Add(solid);
            feedback.Refresh();
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
            object? o = TakeResult();
            Project? project = FrameImpl.MainFrame?.Project;
            Style? style = null;
            if (project != null)
            {
                style = project.StyleList.GetDefault(Style.EDefaultFor.Solids);
            }
            if (o is Solid sld)
            {
                if (style != null) { sld.Style = style; }
                project?.GetActiveModel()?.Add(sld);
            }
            else if (o is List<Solid> solids)
            {
                for (int i = 0; i < solids.Count; i++)
                {
                    if (style != null) { solids[i].Style = style; }
                    project?.GetActiveModel()?.Add(solids[i]);
                }
            }
            base.OnDone();
        }

        /// <summary>
        /// What to put into the model. The preview has usually already computed exactly this, so the common
        /// case costs nothing. Otherwise the template runs once more - with dialogs enabled, so an invalid
        /// combination reports itself in a message box the way it does everywhere else in the application.
        /// </summary>
        private object? TakeResult()
        {
            TemplatePreviewResult? current = lastResult;
            preview?.Dispose(); // nothing computed from here on can still matter
            preview = null;
            if (current != null && current.Error == null && current.Solids.Count > 0
                && SameValues(current.Parameters, parameterValues))
            {
                return current.Solids;
            }
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
