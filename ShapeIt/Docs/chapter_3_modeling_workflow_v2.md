# 3. Modeling Workflow

## 3.1 Typical RPC Script Structure

A ShapeIt modeling script is typically organized as a sequence of steps.
It behaves similarly to a small program:

1. Parameter definition
2. Sketch creation
3. Sketch geometry construction
4. Solid creation
5. Boolean operations
6. Feature operations
7. Commit

---

## 3.2 Parametric Modeling Strategy

ShapeIt is fundamentally parametric. Good models are driven by parameters.

- define parameters early
- use expressions instead of constants
- reuse existing objects

---

## 3.3 Robust Modeling

A robust model continues to work even when parameters change.

- avoid fragile references
- prefer queries
- use bounding boxes
- validate assumptions

---

## 3.4 Templates and Reuse

Templates introduce a higher level of structure.

A template is comparable to a function:

- it defines parameters
- it contains modeling steps
- it produces geometry

### Defining and using templates

A template is defined once and can be instantiated multiple times with different arguments.

This enables:

- reuse of complex constructions
- separation of definition and usage
- cleaner modeling scripts

### Templates as reusable assets

Templates are not only a modeling convenience, but also a **valuable reusable asset**.

Users can store templates and build a **template library** containing frequently used components such as standard parts, connectors, or construction elements.

For this reason, templates should be defined more thoroughly than ad-hoc geometry:

- provide meaningful `label` and `description`
- organize templates using `category` and `tags`
- define parameters clearly using `TemplateParameterSpec`
  - including default values
  - and optional UI metadata (input type, ranges, grouping)

This improves usability, discoverability, and integration into user interfaces.

### Parameters and placement

Template instantiation typically involves two aspects:

1. **Parameterization**  
   Values for the template parameters are provided via `arguments`.

2. **Placement**  
   The instantiated geometry is positioned using a **transformation**.

In practice, placement is almost always required, since templates are usually defined in a local coordinate system.

```json
{"method":"template.instantiate","params":{
  "template":"component",
  "arguments":{"size":50},
  "transform":"translate(x,y,z)"
}}
```

This separation of **definition**, **parameterization**, and **placement** is a key concept when working with templates.

---

## 3.5 Combining Workflow and Templates

In practice, modeling combines both approaches:

- linear workflow
- structured workflow with templates

---

## 3.6 Summary

A good ShapeIt modeling workflow is:

- structured
- parametric
- robust
- reusable
