# 2. Basic Concepts

## 2.1 Workspace Concept

All operations in ShapeIt are executed within a **workspace**.

The workspace is a shared environment that stores:

-   named objects (sketches, solids, surfaces)
-   variables defined by the user
-   intermediate results
-   computed values

The workspace persists during batch execution and also across multiple
batches in an interactive session.

Objects are typically created with a `name` parameter and can later be
referenced by that name.

------------------------------------------------------------------------

## 2.2 Expressions

Many parameters in ShapeIt are not limited to constant values.
Instead, they can be defined using **expressions**.

Expressions allow:

-   mathematical operations
-   references to variables
-   access to object properties
-   evaluation of functions

### Basic usage

``` json
{"method":"workspace.set","params":{"name":"length","value":100}}
{"method":"workspace.set","params":{"name":"half","value":"length/2"}}
```

Expressions are evaluated at runtime and can depend on previously
defined values.

------------------------------------------------------------------------

### Math functions

All standard `Math.*` functions are available.

Typical examples:

-   `sin`, `cos`, `tan`
-   `sqrt`, `pow`
-   `min`, `max`
-   `ceil`, `floor`, `abs`

Example:

``` json
{"method":"workspace.set","params":{"name":"count","value":"ceil(width/spacing)"}}
```

------------------------------------------------------------------------

### Accessing object properties

Expressions can access properties of geometry and topology.

#### Face

-   `edgeCount`
-   `surfaceType`
-   `bounds`
-   `distance(point)` *(signed distance)*

#### Edge

-   `curveType`
-   `startPoint`, `endPoint`
-   `pointAt(u)`
-   `directionAt(u)`
-   `startDirection`, `endDirection`
-   `bounds`
-   `length`

#### Solid

-   `volume`
-   `faceCount`

Example:

``` json
{"method":"workspace.set","params":{"name":"vol","value":"mySolid.volume"}}
```

------------------------------------------------------------------------

### Additional behavior

-   expressions are **case-insensitive**
-   automatic conversion between `int` and `double`
-   expressions can return **boolean values**

Boolean expressions are mainly used in queries (see Chapter 10).

------------------------------------------------------------------------

## 2.3 Coordinate Systems

ShapeIt uses multiple coordinate systems:

-   global coordinate system
-   sketch coordinate system
-   local axes in operations

------------------------------------------------------------------------

## 2.4 Object References

Objects in ShapeIt are referenced by **name**.

### Names

Tools reference workspace objects by a single name or an array of names.

### Selections (workspace.select)

To reference objects that have no name yet — e.g. "the four vertical edges
of this box" or "all faces on the top side" — first call `workspace.select`.
It executes a query (with filters such as `extreme` or `onFace`) or a set
operation (`union`, `difference`, `intersect`) and stores the result under a
name, which subsequent tools can use like any other workspace name.

A selection is a snapshot: modifying a solid afterwards does not update a
previously selected set of its faces or edges, so select right before use.

------------------------------------------------------------------------

## 2.5 Geometry vs Topology

ShapeIt distinguishes between:

### Geometry

The mathematical description of shape.

### Topology

The structural elements:

-   faces
-   edges
-   vertices

------------------------------------------------------------------------

## 2.6 Parametric Modeling

ShapeIt is fundamentally **parametric**.

Models are driven by parameters defined in the workspace.

### User-accessible parameters

Parameters can include metadata such as:

-   `label`
-   `input` definitions (unit, range, grouping, etc.)

These parameters can be exposed to a user interface where:

-   parameters are listed
-   current values are displayed
-   users can modify them interactively

------------------------------------------------------------------------

## 2.7 Naming and Reuse

Naming is a key concept in ShapeIt.

Best practices:

-   use meaningful names
-   reuse objects instead of recreating them
-   structure scripts logically

### Direct name usage

Names can often be used directly without wrapping them in objects:

``` json
{..."method":"solid.boolean","params":{
"op":"difference",
"a":"pipe",
"b":["small_holes","big_holes"],
"name":"perforated_pipe"}}
```
is equivalent to 

``` json
{..."method":"solid.boolean","params":{
"op":"difference",
"a":{ "name": "pipe" },
"b":[{"name":"small_holes"},{"name":"big_holes"}],
"name":"perforated_pipe"}}
```


Names can also represent **lists of objects**, which allows compact and
readable definitions.

------------------------------------------------------------------------

## 2.8 Summary

The core concepts of ShapeIt are:

-   workspace
-   expressions
-   coordinate systems
-   object references
-   geometry vs topology
-   parametric modeling

------------------------------------------------------------------------

## 2.9 Expression Guidelines and Capabilities

Expressions in ShapeIt are a central mechanism for defining parametric behavior.
They are not limited to simple numeric formulas but support multiple data types,
geometry access, and transformations.

------------------------------------------------------------------------

### Supported types

Expressions can evaluate to the following types:

-   `int`, `double`
-   `boolean`
-   `point2`, `point3`
-   `vector2`, `vector3`
-   `bounding box` (2D and 3D)
-   `transformation` (affine transformation: rotation, translation, scaling)

Type conversion between `int` and `double` is automatic.

------------------------------------------------------------------------

### Geometry access

Expressions can access properties of existing objects.

#### Sketch geometry

-   `line.startPoint`, `line.endPoint`
-   `circle.center`, `circle.radius`
-   `arc.startAngle`, `arc.endAngle`

Example:

```json
{"method":"workspace.set","params":{"name":"cx","value":"circle.center.x"}}
```

#### Solid properties

-   `solid.bounds`
-   `solid.volume`

Example:

```json
{"method":"workspace.set","params":{"name":"height","value":"mySolid.bounds.zDiff"}}
```

------------------------------------------------------------------------

### Point and vector construction

Points and vectors can be constructed explicitly:

-   `p(x,y,z)` → point
-   `v(x,y,z)` → vector

Basic operations:

-   `point - point` → vector
-   `point + vector` → point
-   `vector + vector` → vector

Not defined:

-   `point + point`

Vector operations:

-   `v1 * v2` → dot product
-   `v1 ^ v2` → cross product

------------------------------------------------------------------------

### Transformations

Transformations are first-class values in expressions.

They represent affine transformations (matrix + translation).

#### Construction

-   `move(x,y,z)` or `move(v)`
-   `translate(x,y,z)` (synonym)
-   `rotate(p, v, a)`  
    (center point, axis direction, angle in radians)
-   `scale(fx, fy, fz)`
-   `scale(p, f)`

#### Composition

Transformations can be multiplied:

-   `T1 * T2`

Order rule:

-   **right side is applied first**
-   **left side is applied last**

#### Application

-   `T * point` → transformed point
-   `T * vector` → transformed vector (translation ignored)

------------------------------------------------------------------------

### Functions

All standard `Math.*` functions are available:

-   `sin`, `cos`, `tan`
-   `sqrt`, `pow`
-   `min`, `max`
-   `abs`, `floor`, `ceil`

Notes:

-   angles are always in **radians**
-   `min` and `max` accept multiple arguments

Additional functions:

-   `distance(p1, p2)`
-   `normalize(v)`

---

### Boolean expressions

Expressions can return boolean values:

```json
"condition": "this.bounds.zDiff > 10 && this.volume < 1000"
```

They are mainly used in:

-   queries
-   pattern conditions
-   assertions

------------------------------------------------------------------------

### Approximate comparison

Floating-point comparisons using `==` are exact and therefore often not suitable
for geometric or numeric evaluations.

For tolerance-based comparisons, use:

-   `near(a, b)`
-   `near(a, b, tol)`

These functions return `true` if the values are approximately equal.

#### Numeric values

-   `near(a, b)`  
    Uses a default tolerance.

-   `near(a, b, tol)`  
    Uses an explicit tolerance.

Example:

```json
{"method":"workspace.set","params":{"name":"isEqual","value":"near(a, b)"}}
```
#### Points and vectors

near can also be used for geometric types:

-   `near(p1, p2)`

-   `near(v1, v2)`

The comparison is based on the distance (or vector length of the difference):

-   `|p1 - p2| < tol`

-   `|v1 - v2| < tol`

Example:

```json
{"method":"workspace.set","params":{"name":"samePoint","value":"near(p1, p2, 0.001)"}}
```

#### Notes
-   The default tolerance is chosen to be suitable for typical modeling tasks.
-   Use an explicit tolerance for critical comparisons.
-   Prefer near over manual expressions like `abs(a-b) < tol` for readability.

------------------------------------------------------------------------


### General recommendations

-   avoid hardcoded values
-   derive values from geometry when possible
-   keep expressions readable
-   prefer named intermediate variables (`workspace.set`)
-   use geometry properties instead of duplicating dimensions

------------------------------------------------------------------------

### Important consistency note

There is a deliberate distinction between:

-   **Expressions → radians**
-   **RPC parameters → degrees**

This must be considered when mixing both contexts:

```json
{"method":"workspace.set","params":{"name":"angleRad","value":"30*pi/180"}}
```

------------------------------------------------------------------------

### Summary

Expressions provide:

-   numeric computation
-   geometric reasoning
-   transformation logic
-   conditional evaluation

They are a key element for building fully parametric and reusable models.
