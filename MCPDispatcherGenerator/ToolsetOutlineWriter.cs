using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;

/// <summary>
/// Generates a readable text outline from an MCP/JSON-RPC toolset definition file.
/// It prints methods and their inputSchema parameters, and expands $ref targets (#/types/*).
/// </summary>
public static class ToolsetOutlineWriter
{
    public static void Generate(string toolsetJsonPath, string outputTxtPath)
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(toolsetJsonPath));
        var root = doc.RootElement;

        var types = root.TryGetProperty("types", out var typesEl) && typesEl.ValueKind == JsonValueKind.Object
            ? typesEl
            : default;

        var sb = new StringBuilder();
        sb.AppendLine("Readable MCP Toolset Definition");
        sb.AppendLine($"Source: {toolsetJsonPath}");
        sb.AppendLine();

        if (!root.TryGetProperty("tools", out var methodsEl) || methodsEl.ValueKind != JsonValueKind.Array)
            throw new InvalidDataException("Expected root.methods to be an array.");

        foreach (var method in methodsEl.EnumerateArray())
        {
            var name = GetString(method, "name") ?? "unknown_method";
            var desc = GetString(method, "description");

            sb.AppendLine();
            sb.AppendLine(name);
            //if (!string.IsNullOrWhiteSpace(desc))
            //    sb.AppendLine($"   // {desc}");

            //if (method.TryGetProperty("inputSchema", out var inputSchema))
            //{
            //    DescribeSchema(inputSchema, sb, indent: 1, types, new HashSet<string>());
            //}
        }

        File.WriteAllText(outputTxtPath, sb.ToString(), Encoding.UTF8);
    }

    private static void DescribeSchema(
        JsonElement schema,
        StringBuilder sb,
        int indent,
        JsonElement types,
        HashSet<string> refStack)
    {
        if (schema.ValueKind != JsonValueKind.Object)
            return;

        // $ref expansion
        if (schema.TryGetProperty("$ref", out var refEl) && refEl.ValueKind == JsonValueKind.String)
        {
            var r = refEl.GetString()!;
            //sb.AppendLine($"{Indent(indent)}-> {r}");

            if (r.StartsWith("#/types/", StringComparison.Ordinal))
            {
                var typeName = r.Substring("#/types/".Length);
                if (types.ValueKind == JsonValueKind.Object && types.TryGetProperty(typeName, out var resolved))
                {
                    // avoid infinite recursion in cyclic refs
                    if (refStack.Add(typeName))
                    {
                        DescribeSchema(resolved, sb, indent + 1, types, refStack);
                        refStack.Remove(typeName);
                    }
                    else
                    {
                        sb.AppendLine($"{Indent(indent + 1)}// (ref cycle detected: {typeName})");
                    }
                }
            }
            return;
        }

        // Print description if present
        //var desc = GetString(schema, "description");
        //if (!string.IsNullOrWhiteSpace(desc))
        //    sb.AppendLine($"{Indent(indent)}// {desc}");

        var type = GetString(schema, "type");

        if (type == "object")
        {
            if (schema.TryGetProperty("properties", out var props) && props.ValueKind == JsonValueKind.Object)
            {
                foreach (var prop in props.EnumerateObject())
                {
                    var propName = prop.Name;
                    var propSchema = prop.Value;

                    var propDesc = GetString(propSchema, "description");
                    var line = $"{Indent(indent)}- {propName}";
                    //if (!string.IsNullOrWhiteSpace(propDesc))
                    //    line += $" // {propDesc}";
                    sb.AppendLine(line);

                    DescribeSchema(propSchema, sb, indent + 1, types, refStack);
                }
            }

            // Optional: show required
            //if (schema.TryGetProperty("required", out var req) && req.ValueKind == JsonValueKind.Array)
            //{
            //    var reqNames = req.EnumerateArray()
            //        .Where(x => x.ValueKind == JsonValueKind.String)
            //        .Select(x => x.GetString())
            //        .Where(x => !string.IsNullOrWhiteSpace(x))
            //        .ToArray();

            //    if (reqNames.Length > 0)
            //        sb.AppendLine($"{Indent(indent)}// required: {string.Join(", ", reqNames)}");
            //}

            // Optional: show oneOf/allOf/anyOf branches
            //PrintCombinator(schema, "oneOf", sb, indent, types, refStack);
            //PrintCombinator(schema, "anyOf", sb, indent, types, refStack);
            //PrintCombinator(schema, "allOf", sb, indent, types, refStack);
        }
        else if (type == "array")
        {
            sb.AppendLine($"{Indent(indent)}- items (array)");
            if (schema.TryGetProperty("items", out var items))
                DescribeSchema(items, sb, indent + 1, types, refStack);

            // Optional: minItems/maxItems
            //if (schema.TryGetProperty("minItems", out var minItems) && minItems.ValueKind == JsonValueKind.Number)
            //    sb.AppendLine($"{Indent(indent)}// minItems: {minItems}");
            //if (schema.TryGetProperty("maxItems", out var maxItems) && maxItems.ValueKind == JsonValueKind.Number)
            //    sb.AppendLine($"{Indent(indent)}// maxItems: {maxItems}");
        }
        else if (!string.IsNullOrWhiteSpace(type))
        {
            //var line = $"{Indent(indent)}- type: {type}";
            //if (schema.TryGetProperty("enum", out var enumEl) && enumEl.ValueKind == JsonValueKind.Array)
            //{
            //    var vals = enumEl.EnumerateArray()
            //        .Select(e => e.ValueKind == JsonValueKind.String ? e.GetString() : e.ToString())
            //        .ToArray();
            //    line += $" enum=[{string.Join(", ", vals)}]";
            //}
            //if (schema.TryGetProperty("default", out var defEl))
            //    line += $" default={defEl}";
            //sb.AppendLine(line);
        }
    }

    private static void PrintCombinator(
        JsonElement schema,
        string name,
        StringBuilder sb,
        int indent,
        JsonElement types,
        HashSet<string> refStack)
    {
        if (!schema.TryGetProperty(name, out var arr) || arr.ValueKind != JsonValueKind.Array)
            return;

        int i = 0;
        foreach (var branch in arr.EnumerateArray())
        {
            sb.AppendLine($"{Indent(indent)}// {name}[{i}]");
            DescribeSchema(branch, sb, indent + 1, types, refStack);
            i++;
        }
    }

    private static string? GetString(JsonElement obj, string propName)
    {
        if (obj.ValueKind == JsonValueKind.Object &&
            obj.TryGetProperty(propName, out var el) &&
            el.ValueKind == JsonValueKind.String)
            return el.GetString();
        return null;
    }

    private static string Indent(int level) => new string(' ', level * 3);
}