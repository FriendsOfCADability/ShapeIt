using System;
using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.UserInterface;
using System.Collections;
using System.Collections.Generic;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    internal class RpcTemplateAction : ConstructAction
    {
        private MCPServer mcpServer;
        private string templateName;
        Dictionary<string, object> parameterValues;

        public RpcTemplateAction(MCPServer mcpServer, string templateName)
        {
            this.mcpServer = mcpServer;
            this.templateName = templateName;
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
                                    li.SetLengthEvent += l => { parameterValues[name] = l; return true; };
                                    li.GetLengthEvent += () => Convert.ToDouble(parameterValues[name]);
                                    li.Optional = true;
                                    inputs.Add(li);
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
                                    IntInput.SetIntEvent += l => { parameterValues[name] = l; };
                                    IntInput.GetIntEvent += () => Convert.ToInt32(parameterValues[name]);
                                    IntInput.Optional = true;
                                    inputs.Add(IntInput);
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
                                    AngleInput.SetAngleEvent += l => { parameterValues[name] = l.Degree; return true; };
                                    AngleInput.GetAngleEvent += () => Angle.Deg(Convert.ToDouble(parameterValues[name]));
                                    AngleInput.Optional = true;
                                    inputs.Add(AngleInput);
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
        }
        public override void OnDone()
        {
            object? o = mcpServer.ExecuteTemplate(templateName, parameterValues);
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
    }
}