# 1. Introduction

## 1.1 What is ShapeIt RPC Modeling

ShapeIt exposes geometric modeling operations as RPC tools.
Each tool performs a well-defined action such as creating a sketch,
generating a solid, performing a boolean operation, or computing a property.

Instead of using a traditional CAD user interface, models are created
by generating structured sequences of RPC calls.

This approach enables deterministic, scriptable, and reproducible modeling workflows.

## 1.2 Why RPC instead of a traditional CAD UI

Compared to interactive CAD systems, RPC-based modeling offers:

- full reproducibility
- parametric control via expressions
- easy automation
- suitability for AI-driven workflows

Rather than manually clicking operations, the entire modeling process
is explicitly defined as a sequence of steps.

## 1.3 Execution Model Overview

Although each operation is technically an individual RPC call,
ShapeIt is not intended to be used as isolated tool invocations.

Instead, modeling is based on **batches of RPC calls** that together
describe a complete modeling task.

A batch behaves like a small program:

1. Parameters are defined  
2. Sketches are created  
3. Geometry is generated  
4. Solids are constructed  
5. Boolean operations combine shapes  
6. Features are applied  
7. The final result is committed  
8. Optional: reusable parts are encapsulated as templates and instantiated where needed

## 1.4 Shared Workspace

All RPC calls inside a batch operate in a shared workspace.

The workspace provides:

- named objects
- intermediate geometry
- parameter variables
- computed values

Objects created in earlier steps remain available for later steps.

## 1.5 Error Handling

Execution follows a simple rule:

**Execution stops at the first error.**

If an RPC call fails:

- the batch is aborted
- the error is returned
- no further steps are executed

## 1.6 Interactive AI-Assisted Workflow

Even though execution is batch-based, modeling can be interactive.

A typical workflow is a dialog between user and AI:

1. Initial concept (e.g. enclosure)
2. Functional features (holes, slots)
3. Mechanical details (snap fits, guides)
4. Finishing features (fillets, chamfers)

Each step builds on the previous result.
As models grow in complexity, parts of the workflow can be encapsulated into reusable templates to simplify further iterations.

## 1.7 Incremental Modeling

Because the workspace persists, modeling can proceed incrementally.

Each new batch can:

- reuse existing geometry
- extend the model
- refine parameters
- add features

This enables flexible and iterative design processes.

## 1.8 Templates and Reusable Modeling

In addition to incremental modeling, ShapeIt supports **reusable modeling logic** through templates.

A template defines a **parametric modeling procedure** that can be instantiated multiple times with different arguments.

Conceptually, a template is similar to a function:

- it defines parameters
- it contains a sequence of modeling steps
- it produces one or more result objects

### Role of templates in the workflow

Templates extend the batch-based workflow:

1. Define a template once (geometry + parameters)
2. Instantiate it multiple times with different values
3. Combine the results using transformations and Boolean operations

This allows:

- reuse of complex geometry definitions
- clean separation between definition and usage
- building higher-level structures from smaller parametric components

### Example usage pattern

A typical workflow using templates:

- define a parametric component (e.g. socket, gear, connector)
- instantiate it multiple times
- position instances using transformations
- combine them into a final model

Templates are especially useful for:

- repeated features
- parametric assemblies
- structured design systems

## 1.9 Typical Use Cases

ShapeIt RPC modeling is especially suitable for:

- parametric modeling
- script-based CAD workflows
- AI-driven modeling (MCP clients)
- automated geometry generation

## 1.10 Summary

ShapeIt combines a tool-based RPC interface with a batch-oriented execution model.

Key characteristics:

- modeling via RPC tools
- batch-based execution
- shared workspace
- fail-fast error handling
- incremental and interactive workflows
