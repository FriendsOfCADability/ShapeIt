# 4. Sketch Modeling

## 4.1 Creating sketches

Sketches are the foundation of most modeling operations. They define 2D geometry in a specific plane, which can later be used to generate solids or guide other operations.

### sketch.create

Creates a new sketch on a specified plane.

Typical usage:

- standard planes: XY, YZ, XZ
- custom planes via origin and axes

Example:

```json
{"method":"sketch.create","params":{"plane":{"standard":"XY"},"name":"base_sketch"}}
```

### sketch.create_on_face

Creates a sketch directly on a face of an existing solid.

This is especially useful for feature-based modeling such as holes, cutouts, or ribs.

Key idea:

- the sketch inherits the local coordinate system of the face
- geometry is defined relative to that face

---

## 4.2 Sketch primitives

Sketch primitives are the basic building blocks of 2D geometry.

### Lines

Defined by start and end points.

```json
{"method":"sketch.add_line","params":{"sketch":{"name":"s"},"start":[0,0],"end":[10,0]}}
```

### Arcs

Defined by center/radius/angles or by three points.

### Circles

Defined by center and radius or diameter.

### Rectangles

Typically constructed from four lines or via helper methods.

### Polygons

Defined by a sequence of vertices.

### Slots

Useful for mechanical designs, typically composed of lines and arcs.

---

## 4.3 Advanced sketch geometry

### NURBS curves

NURBS allow smooth and flexible curve definitions.

They can be defined by:

- control points
- through-points
- parametric expressions

### Polycurves

Polycurves combine multiple curve segments (lines, arcs) into one logical entity.

They are especially useful for:

- complex profiles
- parametric outlines
- extrusion-ready contours

### Text

Text can be converted into sketch geometry.

Typical use cases:

- engraving
- embossing
- labeling

---

## 4.4 Sketch operations

Sketch operations transform or combine existing sketch geometry.

### Boolean

Combine or subtract sketch regions.

- union
- difference
- intersection

### Offset

Creates parallel geometry at a given distance.

Supports:

- open and closed geometry
- region creation (makeRegion)

### Connect

Combines multiple curves into a single continuous profile.

Important:

- curve order does not matter
- endpoints are matched within a tolerance

### Round vertices

Rounds sharp corners in sketch geometry.

Typically used to:

- create fillets in 2D
- prepare profiles for extrusion

### Solid section

Intersects one or more solids with the sketch plane and produces closed section profiles.

This is useful for:

- reverse engineering geometry
- deriving profiles from existing solids
- inspection and analysis

### Vertex extraction

Extracts points from sketch geometry.

Typical use cases:

- reuse of key points
- driving parametric constructions
- measurements

---
## 4.5 Geometric construction and constraint-based helpers

In addition to basic primitives, ShapeIt provides a set of methods that allow
**geometric constructions and intermediate calculations** inside a sketch.

These methods do not primarily create final geometry, but instead help compute
points, directions, or derived curves that can be reused to build robust and
parametric sketches.

This enables a workflow similar to classical geometric construction.

### Constraint-based primitives

Some sketch primitives can be defined using geometric constraints instead of
explicit coordinates.

Examples:

- `sketch.add_circle_by_constraints`  
  Creates a circle defined by tangency conditions and/or points.

- `sketch.add_line_by_constraints`  
  Creates a line based on tangency to existing curves or through a given point.

These methods are especially useful when:

- exact geometric relations are required (e.g. tangency)
- parametric robustness is important
- manual calculation of coordinates should be avoided

---

### Geometric helper methods

The following methods provide geometric constructions that can be used as
building blocks for more complex sketches:

- `foot_point_on_curve`  
  Computes the perpendicular projection of a point onto a curve.

- `intersections`  
  Computes intersection points between curves.

- `perpendicular_through`  
  Constructs a line perpendicular to a given curve or direction through a point.

- `angle_bisector`  
  Computes the bisector of an angle defined by two lines or directions.

- `set_curve_endpoints`  
  Trims or extends curves by redefining their start and end points.  
  Often used together with intersection or constraint results.

---

### Typical workflow

A common pattern is:

1. Create helper geometry or reference curves  
2. Compute key points using geometric helper methods  
3. Use these points to define final sketch primitives  
4. Optionally remove or ignore helper geometry

This approach leads to:

- more readable constructions
- better parametric behavior
- reduced dependency on manually calculated values

---

### Conceptual idea

Instead of computing coordinates numerically, geometry is constructed by
**relations**:

- tangency
- perpendicularity
- intersection
- projection

This makes sketches:

- more robust under parameter changes
- closer to classical CAD construction workflows
- easier to understand and maintain

---

## 4.6 Additional notes

### Parametric usage

All sketch geometry can be driven by expressions.

This enables:

- fully parametric sketches
- dependency on workspace variables
- dynamic updates

### Robustness

To create stable sketches:

- avoid unnecessary dependencies
- prefer geometric relations over hard-coded values
- use tolerances where appropriate

---

## 4.6 Summary

Sketch modeling in ShapeIt provides:

- a flexible 2D construction environment
- a wide range of primitives and advanced geometry
- powerful operations for combining and transforming geometry

Sketches are the starting point for many modeling workflows and play a central role in parametric design.
