"""Generates small synthetic STEP assemblies for StepAssemblyTests.

The part is an axis parallel box (0,0,0)-(10,20,40) mm. The assemblies differ in the way
the placement of an occurrence is written:
- the order of rep_1/rep_2 in the REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION
  (child first like Inventor/Creo/CATIA, or parent first like SolidWorks),
- an identity or a non identity placement in the child representation,
- the order of the items in the ITEM_DEFINED_TRANSFORMATION,
- one or two assembly levels,
- the length unit of each representation context (mm, m, inch, ...),
- an instance by MAPPED_ITEM instead of an assembly,
- several parts whose representations share one representation context,
- the names of the occurrences (instance names or generic names).
All coordinates are defined in mm and written in the unit of their context, so the expected
result doesn't depend on the units.
Run it with python 3 in this directory; it also prints the expected bounding boxes.
"""
import math

BOX = (10.0, 20.0, 40.0)

# frames in mm: (location, z axis, x axis)
IDENT = ((0, 0, 0), (0, 0, 1), (1, 0, 0))
P1 = ((100, 0, 0), (0, 0, 1), (0, 1, 0))            # rotated 90 degrees about z
P2 = ((0, 50, 25), (1, 0, 0), (0, 0, 1))            # x->z, y->-y, z->x
FC = ((5, 5, 5), (0, 0, 1), (0, 1, 0))              # non identity placement inside the part
Q = ((-30, 7, 3), (0, 1, 0), (1, 0, 0))             # placement of the part in the sub assembly
R = ((200, 0, 0), (0, 0, 1), (1, 0, 0))             # placement of the small cube
SMALL = (5.0, 5.0, 5.0)

# length units: millimetres per unit
MM_PER_UNIT = {"mm": 1.0, "mm_deg": 1.0, "m": 1000.0, "inch": 25.4, "inch_m": 25.4, "mm_m": 1.0}


def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def real(x):
    """A STEP real: 6 decimals when that is exact (keeps the older files unchanged), else full precision."""
    x = float(x)
    if abs(x * 1e6 - round(x * 1e6)) < 1e-6: return f"{x:.6f}"
    s = f"{x:.15g}"
    if 'e' in s:
        m, e = s.split('e')
        return (m if '.' in m else m + '.') + 'E' + e
    return s if '.' in s else s + '.'


class Writer:
    def __init__(self):
        self.lines = []
        self.n = 0

    def add(self, text):
        self.n += 1
        self.lines.append(f"#{self.n}={text};")
        return f"#{self.n}"

    def pt(self, p, unit="mm"): return self.add("CARTESIAN_POINT('',(%s))" % ",".join(real(x / MM_PER_UNIT[unit]) for x in p))
    def dir(self, d): return self.add("DIRECTION('',(%s))" % ",".join(f"{float(x):.6f}" for x in d))

    def ax(self, f, unit="mm"):
        loc, z, x = f
        return self.add(f"AXIS2_PLACEMENT_3D('',{self.pt(loc, unit)},{self.dir(z)},{self.dir(x)})")

    def cto(self, rot, t, unit="mm", scale=None, omit_axis2=False, stretch_axis1=1.0):
        """A CARTESIAN_TRANSFORMATION_OPERATOR_3D p -> t + scale * rot * p, rot given by its columns, t in mm."""
        a1 = self.dir([stretch_axis1 * v for v in rot[0]])
        a2 = "$" if omit_axis2 else self.dir(rot[1])
        a3 = self.dir(rot[2])
        s = "$" if scale is None else real(scale)
        return self.add(f"CARTESIAN_TRANSFORMATION_OPERATOR_3D('','','',{a1},{a2},{self.pt(t, unit)},{s},{a3})")

    def text(self, name):
        hdr = ["ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION((''),'2;1');",
               f"FILE_NAME('{name}','2026-11-21T00:00:00',(''),(''),'make_assemblies.py','CADability tests','');",
               "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));", "ENDSEC;", "DATA;"]
        return "\n".join(hdr + self.lines + ["ENDSEC;", "END-ISO-10303-21;", ""])


def context(w, unit="mm"):
    """A representation context with the length unit `unit`; the non mm contexts use degree as angle unit."""
    if unit == "mm":
        length = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
        angle = w.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    else:
        rad = w.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
        dim = w.add("DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.)")
        deg = w.add(f"PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),{rad})")
        angle = w.add(f"(CONVERSION_BASED_UNIT('DEGREE',{deg})NAMED_UNIT({dim})PLANE_ANGLE_UNIT())")
        if unit == "mm_deg":
            length = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
        elif unit == "m":
            length = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))")
        else:
            ldim = w.add("DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)")
            if unit == "inch":
                base, name, value = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))"), "INCH", "25.4"
            elif unit == "inch_m":
                base, name, value = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))"), "INCH", "0.0254"
            else:  # mm_m
                base, name, value = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.))"), "MILLIMETRE", "0.001"
            conv = w.add(f"LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({value}),{base})")
            length = w.add(f"(CONVERSION_BASED_UNIT('{name}',{conv})LENGTH_UNIT()NAMED_UNIT({ldim}))")
    sr = w.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = w.add(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),{length},'distance_accuracy_value','')")
    return w.add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({unc}))"
                 f"GLOBAL_UNIT_ASSIGNED_CONTEXT(({length},{angle},{sr}))REPRESENTATION_CONTEXT('',''))")


def product(w, appctx, name):
    pc = w.add(f"PRODUCT_CONTEXT('',{appctx},'mechanical')")
    prod = w.add(f"PRODUCT('{name}','{name}','',({pc}))")
    pdf = w.add(f"PRODUCT_DEFINITION_FORMATION('','',{prod})")
    pdc = w.add(f"PRODUCT_DEFINITION_CONTEXT('part definition',{appctx},'design')")
    pd = w.add(f"PRODUCT_DEFINITION('design','',{pdf},{pdc})")
    pds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{pd})")
    return pd, pds


def box_brep(w, unit="mm", size=BOX):
    a, b, c = size
    corners = [(x * a, y * b, z * c) for z in (0, 1) for y in (0, 1) for x in (0, 1)]
    vp = [w.add(f"VERTEX_POINT('',{w.pt(p, unit)})") for p in corners]
    edges = {}

    def edge(i, j):
        key = (min(i, j), max(i, j))
        if key not in edges:
            p, q = corners[key[0]], corners[key[1]]
            d = tuple(qq - pp for pp, qq in zip(p, q))
            ln = math.sqrt(sum(x * x for x in d))
            start = w.pt(p, unit)
            vec = w.add(f"VECTOR('',{w.dir(tuple(x / ln for x in d))},{real(ln / MM_PER_UNIT[unit])})")
            line = w.add(f"LINE('',{start},{vec})")
            edges[key] = w.add(f"EDGE_CURVE('',{vp[key[0]]},{vp[key[1]]},{line},.T.)")
        return w.add(f"ORIENTED_EDGE('',*,*,{edges[key]},{'.T.' if i < j else '.F.'})")

    faces = []
    for axis in range(3):
        for side in (0, 1):
            n = [0, 0, 0]; n[axis] = 1 if side else -1
            idx = [k for k, p in enumerate(corners) if p[axis] == (side * size[axis])]
            u = [0, 0, 0]; u[(axis + 1) % 3] = 1
            v = cross(n, u)
            cen = [sum(corners[k][m] for k in idx) / 4 for m in range(3)]
            idx.sort(key=lambda k: math.atan2(sum((corners[k][m] - cen[m]) * v[m] for m in range(3)),
                                               sum((corners[k][m] - cen[m]) * u[m] for m in range(3))))
            loop = w.add("EDGE_LOOP('',(%s))" % ",".join(edge(idx[m], idx[(m + 1) % 4]) for m in range(4)))
            bound = w.add(f"FACE_OUTER_BOUND('',{loop},.T.)")
            plane = w.add(f"PLANE('',{w.ax((tuple(cen), tuple(n), tuple(u)), unit)})")
            faces.append(w.add(f"ADVANCED_FACE('',({bound}),{plane},.T.)"))
    shell = w.add("CLOSED_SHELL('',(%s))" % ",".join(faces))
    return w.add(f"MANIFOLD_SOLID_BREP('Box',{shell})")


def start():
    w = Writer()
    appctx = w.add("APPLICATION_CONTEXT('automotive design')")
    w.add(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,{appctx})")
    return w, appctx


def write(w, name):
    with open(name, "w", newline="\n") as f:
        f.write(w.text(name))


def make(name, levels, units=None):
    """levels: list of (parent_frame, child_frame, child_first, items_swapped[, cto]) from top to the part.
    cto: None for an ITEM_DEFINED_TRANSFORMATION, else a dict of options for Writer.cto: the placement is then written as
    a CARTESIAN_TRANSFORMATION_OPERATOR_3D, which maps rep_1 into rep_2.
    units: length units of the part and the assemblies from the bottom up (default all mm)."""
    units = units or ["mm"] * (len(levels) + 1)
    w, appctx = start()
    # every product gets its own representation context, as most exporters do (a context shared by the
    # representations of several products makes the importer attach all their items to every product)
    # the part
    unit = units[0]
    ctx = context(w, unit)
    part_pd, part_pds = product(w, appctx, "Box")
    child_frame = levels[-1][1] if levels else IDENT
    part_place = w.ax(child_frame, unit)
    part_sr = w.add(f"SHAPE_REPRESENTATION('Box',({part_place}),{ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({part_pds},{part_sr})")
    brep = box_brep(w, unit)
    absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('Box',({brep},{w.ax(IDENT, unit)}),{ctx})")
    w.add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{part_sr},{absr})")
    child = (part_pd, part_sr, part_place)
    # assemblies from the bottom up
    for k, level in enumerate(reversed(levels)):
        pframe, cframe, child_first, swapped = level[:4]
        cto = level[4] if len(level) > 4 else None
        unit = units[k + 1]
        aname = "Top" if k == len(levels) - 1 else f"Sub{k}"
        apd, apds = product(w, appctx, aname)
        ctx = context(w, unit)
        aplace = w.ax(pframe, unit)
        # the own placement of this assembly as seen from its parent (identity for the top level)
        own = w.ax(levels[len(levels) - 2 - k][1], unit) if k < len(levels) - 1 else w.ax(IDENT, unit)
        asr = w.add(f"SHAPE_REPRESENTATION('{aname}',({own},{aplace}),{ctx})")
        w.add(f"SHAPE_DEFINITION_REPRESENTATION({apds},{asr})")
        cpd, csr, cplace = child
        nauo = w.add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('{aname}-{k}','{aname}-{k}','',{apd},{cpd},$)")
        npds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{nauo})")
        rep1, rep2 = (csr, asr) if child_first else (asr, csr)
        it1, it2 = (cplace, aplace) if child_first else (aplace, cplace)
        if swapped: it1, it2 = it2, it1
        if cto is None:
            idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{it1},{it2})")
        else:
            # rep_1 -> rep_2: child into parent (P C^-1) or parent into child (C P^-1), the origin in the units of rep_2
            src, dst = (cframe, pframe) if child_first else (pframe, cframe)
            rot, t = relative(src, dst)
            idt = w.cto(rot, t, unit if child_first else units[k], **cto)
        rr = w.add(f"(REPRESENTATION_RELATIONSHIP('','',{rep1},{rep2})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt})"
                   f"SHAPE_REPRESENTATION_RELATIONSHIP())")
        w.add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{npds})")
        child = (apd, asr, own)
    write(w, name)


def make_mapped(name, origin, target, outer_unit, inner_unit, target_scale=None):
    """A single product whose shape representation instances the box by a MAPPED_ITEM: the box is in its own
    representation (unit inner_unit) with the mapping origin, the mapping target is in the outer representation.
    With target_scale the target is a CARTESIAN_TRANSFORMATION_OPERATOR_3D with the axes of the target frame."""
    w, appctx = start()
    pd, pds = product(w, appctx, "Box")
    inner_ctx = context(w, inner_unit)
    brep = box_brep(w, inner_unit)
    origin_place = w.ax(origin, inner_unit)
    absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('Box',({brep},{origin_place}),{inner_ctx})")
    rmap = w.add(f"REPRESENTATION_MAP({origin_place},{absr})")
    outer_ctx = context(w, outer_unit)
    if target_scale is None:
        target_place = w.ax(target, outer_unit)
    else:
        rot, t = relative(IDENT, target)
        target_place = w.cto(rot, t, outer_unit, scale=target_scale)
    mapped = w.add(f"MAPPED_ITEM('',{rmap},{target_place})")
    sr = w.add(f"SHAPE_REPRESENTATION('Box',({w.ax(IDENT, outer_unit)},{mapped}),{outer_ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({pds},{sr})")
    write(w, name)


def make_shared_context(name):
    """An assembly "Top" with two different parts, all representations share one representation context:
    "Box" (BOX) placed twice at P1 and P2, "Cube" (SMALL) placed once at R. Every occurrence must get only the
    shape of its own part."""
    w, appctx = start()
    ctx = context(w)
    parts = []
    for pname, size in (("Box", BOX), ("Cube", SMALL)):
        pd, pds = product(w, appctx, pname)
        place = w.ax(IDENT)
        sr = w.add(f"SHAPE_REPRESENTATION('{pname}',({place}),{ctx})")
        w.add(f"SHAPE_DEFINITION_REPRESENTATION({pds},{sr})")
        absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('{pname}',({box_brep(w, size=size)},{w.ax(IDENT)}),{ctx})")
        w.add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{sr},{absr})")
        parts.append((pd, sr, place))
    apd, apds = product(w, appctx, "Top")
    frames = [(parts[0], P1), (parts[0], P2), (parts[1], R)]
    aplaces = [w.ax(f) for _, f in frames]
    asr = w.add("SHAPE_REPRESENTATION('Top',(%s),%s)" % (",".join([w.ax(IDENT)] + aplaces), ctx))
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({apds},{asr})")
    for k, (((cpd, csr, cplace), _), aplace) in enumerate(zip(frames, aplaces)):
        nauo = w.add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('Top-{k}','Top-{k}','',{apd},{cpd},$)")
        npds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{nauo})")
        idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{cplace},{aplace})")
        rr = w.add(f"(REPRESENTATION_RELATIONSHIP('','',{csr},{asr})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt})"
                   f"SHAPE_REPRESENTATION_RELATIONSHIP())")
        w.add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{npds})")
    write(w, name)


def make_occurrence_names(name):
    """An assembly "Top" with a part "Pair" of two solids (BOX and SMALL), placed at P1 with the instance name
    "Pair:1" and at P2 with the generic name "NAUO2", and a part "Box" of one solid, placed at R as "Box:1".
    An occurrence of several objects is a block named after the instance name, if the instance name extends the
    product name, otherwise after the product. A single solid is not put into a block and keeps the product name."""
    w, appctx = start()
    ctx = context(w)
    parts = {}
    for pname, sizes in (("Pair", (BOX, SMALL)), ("Box", (BOX,))):
        pd, pds = product(w, appctx, pname)
        place = w.ax(IDENT)
        sr = w.add(f"SHAPE_REPRESENTATION('{pname}',({place}),{ctx})")
        w.add(f"SHAPE_DEFINITION_REPRESENTATION({pds},{sr})")
        breps = ",".join(box_brep(w, size=s) for s in sizes)
        absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('{pname}',({breps},{w.ax(IDENT)}),{ctx})")
        w.add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{sr},{absr})")
        parts[pname] = (pd, sr, place)
    apd, apds = product(w, appctx, "Top")
    frames = [("Pair", P1, "Pair:1"), ("Pair", P2, "NAUO2"), ("Box", R, "Box:1")]
    aplaces = [w.ax(f) for _, f, _ in frames]
    asr = w.add("SHAPE_REPRESENTATION('Top',(%s),%s)" % (",".join([w.ax(IDENT)] + aplaces), ctx))
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({apds},{asr})")
    for (pname, _, nname), aplace in zip(frames, aplaces):
        cpd, csr, cplace = parts[pname]
        nauo = w.add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('{nname}','{nname}','',{apd},{cpd},$)")
        npds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{nauo})")
        idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{cplace},{aplace})")
        rr = w.add(f"(REPRESENTATION_RELATIONSHIP('','',{csr},{asr})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt})"
                   f"SHAPE_REPRESENTATION_RELATIONSHIP())")
        w.add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{npds})")
    write(w, name)


def make_styled_arc(name):
    """A trimmed circle with parameters in degree in a GEOMETRIC_CURVE_SET, styled by a presentation
    representation whose context is a plain REPRESENTATION_CONTEXT without units (as ThinkDesign writes it).
    The arc must be trimmed in the degree units of its own representation."""
    w, appctx = start()
    pd, pds = product(w, appctx, "Arc")
    ctx = context(w, "mm_deg")
    place = w.add(f"AXIS2_PLACEMENT_3D('',{w.pt((0, 0, 0))},{w.dir((0, 0, 1))},{w.dir(ARC_REF)})")
    circle = w.add(f"CIRCLE('',{place},{real(ARC_RADIUS)})")
    p1, p2 = w.pt(arc_point(ARC_START)), w.pt(arc_point(ARC_END))
    arc = w.add(f"TRIMMED_CURVE('',{circle},(PARAMETER_VALUE({real(ARC_START)}),{p1}),(PARAMETER_VALUE({real(ARC_END)}),{p2}),.T.,.PARAMETER.)")
    gcs = w.add(f"GEOMETRIC_CURVE_SET('',({arc}))")
    rep = w.add(f"GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('Arc',({gcs},{w.ax(IDENT)}),{ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({pds},{rep})")
    colour = w.add("DRAUGHTING_PRE_DEFINED_COLOUR('red')")
    style = w.add(f"CURVE_STYLE('',$,$,{colour})")
    psa = w.add(f"PRESENTATION_STYLE_ASSIGNMENT(({style}))")
    styled = w.add(f"STYLED_ITEM('',({psa}),{arc})")
    plain = w.add("REPRESENTATION_CONTEXT('','')")
    w.add(f"MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',({styled}),{plain})")
    write(w, name)


ARC_RADIUS = 8.0
ARC_REF = (-0.484122918275927, 0.875, 0.0)      # x axis of the circle
ARC_START, ARC_END = 270.0, 302.08997990417     # degree, relative to ARC_REF


def arc_point(deg):
    a = math.radians(deg) + math.atan2(ARC_REF[1], ARC_REF[0])
    return (ARC_RADIUS * math.cos(a), ARC_RADIUS * math.sin(a), 0.0)


def arc_extent():
    pts = [arc_point(ARC_START + (ARC_END - ARC_START) * i / 1000) for i in range(1001)]
    return [min(p[r] for p in pts) for r in range(3)] + [max(p[r] for p in pts) for r in range(3)]


# ---------------------------------------------------------------- expected results

def frame_matrix(f):
    loc, z, x = f
    y = cross(z, x)
    return [[x[r], y[r], z[r]] for r in range(3)], list(loc)


def apply_frame(f, p):
    R, t = frame_matrix(f)
    return [sum(R[r][c] * p[c] for c in range(3)) + t[r] for r in range(3)]


def apply_inverse(f, p):
    R, t = frame_matrix(f)
    d = [p[r] - t[r] for r in range(3)]
    return [sum(R[c][r] * d[c] for c in range(3)) for r in range(3)]


def relative(src, dst):
    """The transformation dst * inverse(src) as (columns of the rotation, translation)."""
    rs, ts = frame_matrix(src)
    rd, td = frame_matrix(dst)
    rot = [[sum(rd[r][i] * rs[c][i] for i in range(3)) for r in range(3)] for c in range(3)]  # columns
    t = [td[r] - sum(rot[c][r] * ts[c] for c in range(3)) for r in range(3)]
    return rot, t


def expected(chain, size=BOX):
    """chain: list of (parent_frame, child_frame) from top to the part; world = P0 C0^-1 P1 C1^-1 ... box"""
    a, b, c = size
    pts = [[x * a, y * b, z * c] for x in (0, 1) for y in (0, 1) for z in (0, 1)]
    for pf, cf in reversed(chain):
        pts = [apply_frame(pf, apply_inverse(cf, p)) for p in pts]
    lo = [min(p[r] for p in pts) for r in range(3)]
    hi = [max(p[r] for p in pts) for r in range(3)]
    return lo + hi


def show(fn, chain): print(fn, [round(v, 6) for v in expected(chain)])


if __name__ == "__main__":
    for child_first in (True, False):
        tag = "child_first" if child_first else "parent_first"
        for cf, ctag in ((IDENT, "ident"), (FC, "offset")):
            for swapped in (False, True):
                fn = f"box_{tag}_{ctag}{'_swapped' if swapped else ''}.stp"
                # two occurrences need two files with one occurrence each to stay simple: P1 and P2
                for pf, ptag in ((P1, "p1"), (P2, "p2")):
                    f2 = fn.replace(".stp", f"_{ptag}.stp")
                    make(f2, [(pf, cf, child_first, swapped)])
                    show(f2, [(pf, cf)])
    # two levels: Top -(P1, child first)-> Sub -(Q, parent first, child placement FC)-> Box
    make("box_two_levels.stp", [(P1, IDENT, True, False), (Q, FC, False, False)])
    show("box_two_levels.stp", [(P1, IDENT), (Q, FC)])
    # units: a single part in different length units
    for unit in ("inch", "m", "inch_m", "mm_m"):
        make(f"box_unit_{unit}.stp", [], [unit])
        show(f"box_unit_{unit}.stp", [])
    # units: part and assembly in different units
    make("box_child_first_offset_p1_part_inch.stp", [(P1, FC, True, False)], ["inch", "mm"])
    show("box_child_first_offset_p1_part_inch.stp", [(P1, FC)])
    make("box_parent_first_offset_p2_assembly_inch.stp", [(P2, FC, False, False)], ["mm", "inch"])
    show("box_parent_first_offset_p2_assembly_inch.stp", [(P2, FC)])
    make("box_two_levels_units.stp", [(P1, IDENT, True, False), (Q, FC, False, False)], ["mm", "inch", "m"])
    show("box_two_levels_units.stp", [(P1, IDENT), (Q, FC)])
    # mapped items
    make_mapped("box_mapped.stp", FC, P1, "mm", "mm")
    show("box_mapped.stp", [(P1, FC)])
    make_mapped("box_mapped_inch.stp", FC, P1, "mm", "inch")
    show("box_mapped_inch.stp", [(P1, FC)])
    make_mapped("box_mapped_outer_m.stp", FC, P1, "m", "mm")
    show("box_mapped_outer_m.stp", [(P1, FC)])
    # cartesian transformation operators instead of item defined transformations; the second one with axis2 omitted
    # and a non normalized axis1, the mapped one scaled by 2
    make("box_cto_child_first_offset_p1_assembly_inch.stp", [(P1, FC, True, False, {})], ["mm", "inch"])
    show("box_cto_child_first_offset_p1_assembly_inch.stp", [(P1, FC)])
    make("box_cto_parent_first_offset_p2_part_inch.stp", [(P2, FC, False, False, {"omit_axis2": True, "stretch_axis1": 3.0})], ["inch", "mm"])
    show("box_cto_parent_first_offset_p2_part_inch.stp", [(P2, FC)])
    make_mapped("box_mapped_cto_scaled.stp", FC, P1, "inch", "mm", target_scale=2.0)
    # P1 * 2 * inverse(FC) is P1 * inverse(FC with twice the location) applied to the box scaled by 2
    print("box_mapped_cto_scaled.stp", [round(v, 6) for v in expected([(P1, ((10, 10, 10), FC[1], FC[2]))], (20.0, 40.0, 80.0))])
    # product structure: several parts share one representation context
    make_shared_context("shared_context.stp")
    for f, size in ((P1, BOX), (P2, BOX), (R, SMALL)):
        print("shared_context.stp", [round(v, 6) for v in expected([(f, IDENT)], size)])
    # names of the occurrences: blocks only for occurrences of several objects
    make_occurrence_names("occurrence_names.stp")
    # angle units: a styled arc in degree, the presentation context has no units
    make_styled_arc("arc_degree_styled.stp")
    print("arc_degree_styled.stp", [round(v, 6) for v in arc_extent()])
