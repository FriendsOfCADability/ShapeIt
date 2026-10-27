using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using System.Text.Json;
using JsonRpcException = ShapeIt.MCPServer.JsonRpcException;

namespace ShapeIt
{
    /// <summary>
    /// A combination of parameter values a template does not accept: the message for the user and the parameters
    /// it is about. An empty list means the message concerns the template as a whole.
    /// </summary>
    public sealed record ParameterProblem(string Message, IReadOnlyList<string> Parameters);

    /// <summary>One entry of a preset list: its label and the values it sets, as written in the template.</summary>
    public sealed record TemplatePresetOption(string Label, IReadOnlyList<KeyValuePair<string, JsonElement>> Values);

    /// <summary>
    /// A list of named value combinations of a template (e.g. the module series of DIN 780): choosing one sets
    /// the parameters it names, which stay editable.
    /// </summary>
    public sealed record TemplatePreset(string Name, string Label, string? Description, IReadOnlyList<TemplatePresetOption> Options);

    /// <summary>One option of an 'enum' input: the value the template gets (a string, number or bool) and its label.</summary>
    public sealed record TemplateEnumOption(object Value, string Label);

    /// <summary>
    /// The rules a template states about its parameter values - 'min' and 'max' of the value inputs and the
    /// 'checks' of template.begin - and its 'presets'. Static and working on a plain dictionary of values, so that
    /// the server checks the values of an instantiation in its workspace while the application checks the values
    /// being edited on the UI thread, without touching the workspace a background preview is working in.
    /// </summary>
    public static class TemplateParameterRules
    {
        /// <summary>Formats the message for a value outside 'min' or 'max': label of the parameter, whether the
        /// minimum was violated, and the limit as text.</summary>
        public delegate string LimitMessage(string label, bool isMinimum, string limit);

        public static string EnglishLimitMessage(string label, bool isMinimum, string limit)
            => isMinimum ? $"{label} must be at least {limit}." : $"{label} must be at most {limit}.";

        private static readonly HashSet<string> numericKinds = new HashSet<string>(StringComparer.Ordinal) { "length", "number", "integer", "angle" };

        /// <summary>
        /// The problems of the given values: every value outside its 'min'/'max' and every check whose condition
        /// is not true. Limits and conditions are expressions over the parameters.
        /// </summary>
        /// <param name="parameters">'parameters' of template.begin</param>
        /// <param name="checks">'checks' of template.begin, may be undefined</param>
        /// <param name="values">the parameter values (in the workspace: all named items)</param>
        /// <param name="limitMessage">formats min/max violations, English when null</param>
        /// <param name="ignoreEvaluationErrors">a limit or condition that cannot be evaluated is no problem - the
        /// application uses it while required objects are not picked yet</param>
        public static List<ParameterProblem> Check(JsonElement parameters, JsonElement checks, Dictionary<string, object> values,
            LimitMessage? limitMessage = null, bool ignoreEvaluationErrors = false)
        {
            limitMessage ??= EnglishLimitMessage;
            List<ParameterProblem> problems = new List<ParameterProblem>();
            if (parameters.ValueKind == JsonValueKind.Array)
            {
                foreach (JsonElement spec in parameters.EnumerateArray())
                {
                    if (spec.ValueKind != JsonValueKind.Object) continue;
                    if (!spec.TryGetProperty("name", out JsonElement nameEl) || nameEl.ValueKind != JsonValueKind.String) continue;
                    if (!spec.TryGetProperty("input", out JsonElement input) || input.ValueKind != JsonValueKind.Object) continue;
                    string name = nameEl.GetString()!;
                    string? kind = input.TryGetProperty("kind", out JsonElement kindEl) && kindEl.ValueKind == JsonValueKind.String ? kindEl.GetString() : null;
                    string label = spec.TryGetProperty("label", out JsonElement labelEl) && labelEl.ValueKind == JsonValueKind.String ? labelEl.GetString()! : name;
                    if (kind == "enum")
                    {   // the steps test the value with 'when' conditions: one that is no option would match none of them
                        if (!values.TryGetValue(name, out object? choice)) continue;
                        List<TemplateEnumOption> options = ReadEnumOptions(input);
                        if (!options.Exists(o => SameEnumValue(o.Value, choice)))
                            problems.Add(new ParameterProblem($"{label} must be one of {string.Join(", ", options.ConvertAll(o => FormatForMessage(o.Value)))}, not {FormatForMessage(choice)}.", new[] { name }));
                        continue;
                    }
                    if (kind != null && !numericKinds.Contains(kind)) continue;
                    if (!values.TryGetValue(name, out object? value) || !TryToDouble(value, out double number)) continue;
                    foreach (bool isMinimum in new[] { true, false })
                    {
                        if (!input.TryGetProperty(isMinimum ? "min" : "max", out JsonElement limitEl)) continue;
                        double limit;
                        try
                        {
                            if (!TryToDouble(EvaluateValue(limitEl, values), out limit)) throw new InvalidOperationException("not a number");
                        }
                        catch (Exception ex)
                        {
                            if (ignoreEvaluationErrors) continue;
                            problems.Add(new ParameterProblem($"The {(isMinimum ? "min" : "max")} of '{name}' cannot be evaluated: {ex.Message}", new[] { name }));
                            continue;
                        }
                        if (isMinimum ? number < limit : number > limit)
                            problems.Add(new ParameterProblem(limitMessage(label, isMinimum, FormatForMessage(limit)), new[] { name }));
                    }
                }
            }
            if (checks.ValueKind == JsonValueKind.Array)
            {
                foreach (JsonElement check in checks.EnumerateArray())
                {
                    if (check.ValueKind != JsonValueKind.Object) continue;
                    string condition = check.TryGetProperty("condition", out JsonElement c) && c.ValueKind == JsonValueKind.String ? c.GetString()! : "";
                    string message = check.TryGetProperty("message", out JsonElement m) && m.ValueKind == JsonValueKind.String ? m.GetString()! : condition;
                    List<string> concerned = new List<string>();
                    if (check.TryGetProperty("parameters", out JsonElement ps) && ps.ValueKind == JsonValueKind.Array)
                    {
                        foreach (JsonElement p in ps.EnumerateArray())
                        {
                            if (p.ValueKind == JsonValueKind.String) concerned.Add(p.GetString()!);
                        }
                    }
                    object? result;
                    try
                    {
                        result = Evaluator.Evaluate(condition, values);
                    }
                    catch (Exception ex)
                    {
                        if (!ignoreEvaluationErrors) problems.Add(new ParameterProblem($"The check '{condition}' cannot be evaluated: {ex.Message}", concerned));
                        continue;
                    }
                    if (result is bool b)
                    {
                        if (!b) problems.Add(new ParameterProblem(ExpandPlaceholders(message, values), concerned));
                    }
                    else if (!ignoreEvaluationErrors)
                    {
                        problems.Add(new ParameterProblem($"The check '{condition}' does not yield true or false.", concerned));
                    }
                }
            }
            return problems;
        }

        /// <summary>
        /// Checks the form of 'presets' and 'checks' when a template is recorded, so that a misspelled parameter
        /// name is reported to the author and not discovered by the user.
        /// </summary>
        /// <exception cref="JsonRpcException">E_INVALID_PARAMS with the first mistake found</exception>
        public static void ValidateDefinition(JsonElement parameters, JsonElement presets, JsonElement checks)
        {
            HashSet<string> names = new HashSet<string>(StringComparer.Ordinal);
            Dictionary<string, string?> kinds = new Dictionary<string, string?>(StringComparer.Ordinal);
            if (parameters.ValueKind == JsonValueKind.Array)
            {
                foreach (JsonElement spec in parameters.EnumerateArray())
                {
                    if (spec.ValueKind == JsonValueKind.Object && spec.TryGetProperty("name", out JsonElement n) && n.ValueKind == JsonValueKind.String)
                    {
                        names.Add(n.GetString()!);
                        kinds[n.GetString()!] = InputString(spec, "kind");
                    }
                }
                foreach (JsonElement spec in parameters.EnumerateArray())
                {
                    if (spec.ValueKind != JsonValueKind.Object || !spec.TryGetProperty("name", out JsonElement n) || n.ValueKind != JsonValueKind.String) continue;
                    string name = n.GetString()!;
                    string? kind = kinds[name];
                    if (kind == "enum")
                    {
                        spec.TryGetProperty("input", out JsonElement input);
                        List<TemplateEnumOption> options = ReadEnumOptions(input);
                        if (options.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", $"The enum parameter '{name}' needs 'options': values (strings, numbers or true/false), or objects with 'value' and 'label'.");
                        for (int i = 0; i < options.Count; i++)
                        {
                            for (int j = 0; j < i; j++)
                            {
                                if (SameEnumValue(options[i].Value, options[j].Value)) throw new JsonRpcException("E_INVALID_PARAMS", $"The enum parameter '{name}' has the option {FormatForMessage(options[i].Value)} twice.");
                            }
                        }
                    }
                    // the solid a face is picked on, and the face a point is put on, must be parameters of that kind
                    if (kind == "face" && InputString(spec, "solid") is string solid && (!kinds.TryGetValue(solid, out string? solidKind) || solidKind != "solid"))
                        throw new JsonRpcException("E_INVALID_PARAMS", $"The face parameter '{name}' names '{solid}' as its solid, which is no 'solid' parameter of the template.");
                    if (InputString(spec, "onFace") is string face)
                    {
                        if (kind != "point3") throw new JsonRpcException("E_INVALID_PARAMS", $"'onFace' is for 'point3' parameters, '{name}' is no such parameter.");
                        if (!kinds.TryGetValue(face, out string? faceKind) || faceKind != "face")
                            throw new JsonRpcException("E_INVALID_PARAMS", $"The point parameter '{name}' names '{face}' as its face, which is no 'face' parameter of the template.");
                    }
                }
            }
            if (presets.ValueKind != JsonValueKind.Undefined && presets.ValueKind != JsonValueKind.Null)
            {
                if (presets.ValueKind != JsonValueKind.Array) throw new JsonRpcException("E_INVALID_PARAMS", "'presets' must be an array.");
                foreach (TemplatePreset preset in ReadPresets(presets))
                {
                    if (preset.Options.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", $"The preset '{preset.Name}' has no options.");
                    foreach (TemplatePresetOption option in preset.Options)
                    {
                        if (option.Values.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", $"The option '{option.Label}' of preset '{preset.Name}' sets no values.");
                        foreach (KeyValuePair<string, JsonElement> value in option.Values)
                        {
                            if (!names.Contains(value.Key)) throw new JsonRpcException("E_INVALID_PARAMS", $"The option '{option.Label}' of preset '{preset.Name}' sets '{value.Key}', which is no parameter of the template.");
                        }
                    }
                }
            }
            if (checks.ValueKind != JsonValueKind.Undefined && checks.ValueKind != JsonValueKind.Null)
            {
                if (checks.ValueKind != JsonValueKind.Array) throw new JsonRpcException("E_INVALID_PARAMS", "'checks' must be an array.");
                foreach (JsonElement check in checks.EnumerateArray())
                {
                    if (check.ValueKind != JsonValueKind.Object
                        || !check.TryGetProperty("condition", out JsonElement c) || c.ValueKind != JsonValueKind.String
                        || !check.TryGetProperty("message", out JsonElement m) || m.ValueKind != JsonValueKind.String)
                        throw new JsonRpcException("E_INVALID_PARAMS", "Every entry of 'checks' needs a 'condition' and a 'message'.");
                    if (check.TryGetProperty("parameters", out JsonElement ps) && ps.ValueKind == JsonValueKind.Array)
                    {
                        foreach (JsonElement p in ps.EnumerateArray())
                        {
                            if (p.ValueKind != JsonValueKind.String || !names.Contains(p.GetString()!))
                                throw new JsonRpcException("E_INVALID_PARAMS", $"The check '{c.GetString()}' names '{p}', which is no parameter of the template.");
                        }
                    }
                }
            }
        }

        /// <summary>The presets of template.begin in their order; entries without a name or options are skipped.</summary>
        public static List<TemplatePreset> ReadPresets(JsonElement presets)
        {
            List<TemplatePreset> res = new List<TemplatePreset>();
            if (presets.ValueKind != JsonValueKind.Array) return res;
            foreach (JsonElement preset in presets.EnumerateArray())
            {
                if (preset.ValueKind != JsonValueKind.Object) continue;
                if (!preset.TryGetProperty("name", out JsonElement n) || n.ValueKind != JsonValueKind.String) continue;
                string name = n.GetString()!;
                string label = preset.TryGetProperty("label", out JsonElement l) && l.ValueKind == JsonValueKind.String ? l.GetString()! : name;
                string? description = preset.TryGetProperty("description", out JsonElement d) && d.ValueKind == JsonValueKind.String ? d.GetString() : null;
                List<TemplatePresetOption> options = new List<TemplatePresetOption>();
                if (preset.TryGetProperty("options", out JsonElement opts) && opts.ValueKind == JsonValueKind.Array)
                {
                    foreach (JsonElement option in opts.EnumerateArray())
                    {
                        if (option.ValueKind != JsonValueKind.Object) continue;
                        List<KeyValuePair<string, JsonElement>> optionValues = new List<KeyValuePair<string, JsonElement>>();
                        if (option.TryGetProperty("values", out JsonElement vs) && vs.ValueKind == JsonValueKind.Object)
                        {
                            foreach (JsonProperty v in vs.EnumerateObject()) optionValues.Add(new KeyValuePair<string, JsonElement>(v.Name, v.Value));
                        }
                        string optionLabel = option.TryGetProperty("label", out JsonElement ol) && ol.ValueKind == JsonValueKind.String
                            ? ol.GetString()! : string.Join(", ", optionValues.ConvertAll(v => v.Value.ToString()));
                        options.Add(new TemplatePresetOption(optionLabel, optionValues));
                    }
                }
                res.Add(new TemplatePreset(name, label, description, options));
            }
            return res;
        }

        /// <summary>A string property of the 'input' of a parameter, null when there is none.</summary>
        public static string? InputString(JsonElement parameterSpec, string property)
        {
            if (parameterSpec.ValueKind != JsonValueKind.Object || !parameterSpec.TryGetProperty("input", out JsonElement input) || input.ValueKind != JsonValueKind.Object) return null;
            return input.TryGetProperty(property, out JsonElement el) && el.ValueKind == JsonValueKind.String ? el.GetString() : null;
        }

        /// <summary>
        /// The 'options' of an 'enum' input in their order. An option is either the value itself or an object with
        /// 'value' and 'label'; without a label the value is shown. Entries of another form are skipped.
        /// </summary>
        public static List<TemplateEnumOption> ReadEnumOptions(JsonElement input)
        {
            List<TemplateEnumOption> res = new List<TemplateEnumOption>();
            if (input.ValueKind != JsonValueKind.Object || !input.TryGetProperty("options", out JsonElement options) || options.ValueKind != JsonValueKind.Array) return res;
            foreach (JsonElement option in options.EnumerateArray())
            {
                JsonElement valueEl = option;
                string? label = null;
                if (option.ValueKind == JsonValueKind.Object)
                {
                    if (!option.TryGetProperty("value", out valueEl)) continue;
                    if (option.TryGetProperty("label", out JsonElement l) && l.ValueKind == JsonValueKind.String) label = l.GetString();
                }
                if (EnumValue(valueEl) is object value) res.Add(new TemplateEnumOption(value, label ?? FormatForMessage(value)));
            }
            return res;
        }

        /// <summary>The value of an enum option as written: a string, number or bool, null for anything else.</summary>
        private static object? EnumValue(JsonElement value)
        {
            switch (value.ValueKind)
            {
                case JsonValueKind.String: return value.GetString();
                case JsonValueKind.Number: return value.GetDouble();
                case JsonValueKind.True: return true;
                case JsonValueKind.False: return false;
                default: return null;
            }
        }

        /// <summary>
        /// The value of an enum parameter given as text, e.g. the 'value' of template.begin or an argument of
        /// template.instantiate: a text that is one of the string options is that option. Anything else is an
        /// expression, so that "\"chamfer\"" or the name of a value of the caller still work.
        /// </summary>
        public static bool TryEnumLiteral(JsonElement input, string text, out object value)
        {
            foreach (TemplateEnumOption option in ReadEnumOptions(input))
            {
                if (option.Value is string s && s == text)
                {
                    value = s;
                    return true;
                }
            }
            value = null!;
            return false;
        }

        /// <summary>Whether two enum values are the same option: numbers by value, everything else by equality.</summary>
        public static bool SameEnumValue(object? a, object? b)
        {
            if (TryToDouble(a, out double x) && TryToDouble(b, out double y)) return x == y;
            return Equals(a, b);
        }

        /// <summary>A value as written in a template: a number, true/false, or an expression over the given values.</summary>
        public static object EvaluateValue(JsonElement value, Dictionary<string, object> values)
        {
            switch (value.ValueKind)
            {
                case JsonValueKind.Number: return value.GetDouble();
                case JsonValueKind.True: return true;
                case JsonValueKind.False: return false;
                case JsonValueKind.String: return Evaluator.Evaluate(value.GetString()!, values);
                case JsonValueKind.Object when value.TryGetProperty("expr", out JsonElement expr) && expr.ValueKind == JsonValueKind.String:
                    return Evaluator.Evaluate(expr.GetString()!, values);
                default: throw new JsonRpcException("E_INVALID_PARAMS", $"'{value}' is neither a number nor an expression.");
            }
        }

        public static bool TryToDouble(object? value, out double number)
        {
            switch (value)
            {
                case double d: number = d; return true;
                case int i: number = i; return true;
                case float f: number = f; return true;
                case long l: number = l; return true;
                default: number = double.NaN; return false;
            }
        }

        /// <summary>
        /// Replaces every {expression} of a message by its value, "{{" and "}}" by single braces. A placeholder that
        /// cannot be evaluated stays as written.
        /// </summary>
        public static string ExpandPlaceholders(string message, Dictionary<string, object> values)
        {
            if (string.IsNullOrEmpty(message) || message.IndexOf('{') < 0) return message;
            StringBuilder result = new StringBuilder(message.Length + 32);
            for (int i = 0; i < message.Length; i++)
            {
                char c = message[i];
                if (c == '{' && i + 1 < message.Length && message[i + 1] == '{') { result.Append('{'); i++; continue; }
                if (c == '}' && i + 1 < message.Length && message[i + 1] == '}') { result.Append('}'); i++; continue; }
                if (c != '{') { result.Append(c); continue; }
                int end = message.IndexOf('}', i + 1);
                if (end < 0) { result.Append(c); continue; } // unbalanced brace: keep the text as written
                string expression = message.Substring(i + 1, end - i - 1);
                try
                {
                    object value = Evaluator.Evaluate(expression, values);
                    result.Append(value == null ? "{" + expression + "}" : FormatForMessage(value));
                }
                catch (Exception)
                {
                    result.Append("{" + expression + "}");
                }
                i = end;
            }
            return result.ToString();
        }

        /// <summary>A value in a message. Numbers are trimmed to four decimals - a message is read by a human,
        /// the full precision belongs in inspect.properties.</summary>
        public static string FormatForMessage(object value)
        {
            switch (value)
            {
                case double d: return d.ToString("0.####", CultureInfo.InvariantCulture);
                case float f: return f.ToString("0.####", CultureInfo.InvariantCulture);
                default: return Convert.ToString(value, CultureInfo.InvariantCulture) ?? "";
            }
        }
    }
}
