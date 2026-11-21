"""Generates small synthetic STEP assemblies for StepAssemblyTests.

The part is an axis parallel box (0,0,0)-(10,20,40). The assemblies differ in the way
the placement of an occurrence is written:
- the order of rep_1/rep_2 in the REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION
  (child first like Inventor/Creo/CATIA, or parent first like SolidWorks),
- an identity or a non identity placement in the child representation,
- the order of the items in the ITEM_DEFINED_TRANSFORMATION,
- one or two assembly levels.
Run it with python 3 in this directory; it also prints the expected bounding boxes.
"""
import math

BOX = (10.0, 20.0, 40.0)

# frames: (location, z axis, x axis)
IDENT = ((0, 0, 0), (0, 0, 1), (1, 0, 0))
P1 = ((100, 0, 0), (0, 0, 1), (0, 1, 0))            # rotated 90 degrees about z
P2 = ((0, 50, 25), (1, 0, 0), (0, 0, 1))            # x->z, y->-y, z->x
FC = ((5, 5, 5), (0, 0, 1), (0, 1, 0))              # non identity placement inside the part
Q = ((-30, 7, 3), (0, 1, 0), (1, 0, 0))             # placement of the part in the sub assembly


def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class Writer:
    def __init__(self):
        self.lines = []
        self.n = 0

    def add(self, text):
        self.n += 1
        self.lines.append(f"#{self.n}={text};")
        return f"#{self.n}"

    def pt(self, p): return self.add("CARTESIAN_POINT('',(%s))" % ",".join(f"{float(x):.6f}" for x in p))
    def dir(self, d): return self.add("DIRECTION('',(%s))" % ",".join(f"{float(x):.6f}" for x in d))

    def ax(self, f):
        loc, z, x = f
        return self.add(f"AXIS2_PLACEMENT_3D('',{self.pt(loc)},{self.dir(z)},{self.dir(x)})")

    def text(self, name):
        hdr = ["ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION((''),'2;1');",
               f"FILE_NAME('{name}','2026-11-21T00:00:00',(''),(''),'make_assemblies.py','CADability tests','');",
               "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));", "ENDSEC;", "DATA;"]
        return "\n".join(hdr + self.lines + ["ENDSEC;", "END-ISO-10303-21;", ""])


def context(w):
    mm = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    rad = w.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    sr = w.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = w.add(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),{mm},'distance_accuracy_value','')")
    return w.add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({unc}))"
                 f"GLOBAL_UNIT_ASSIGNED_CONTEXT(({mm},{rad},{sr}))REPRESENTATION_CONTEXT('',''))")


def product(w, appctx, name):
    pc = w.add(f"PRODUCT_CONTEXT('',{appctx},'mechanical')")
    prod = w.add(f"PRODUCT('{name}','{name}','',({pc}))")
    pdf = w.add(f"PRODUCT_DEFINITION_FORMATION('','',{prod})")
    pdc = w.add(f"PRODUCT_DEFINITION_CONTEXT('part definition',{appctx},'design')")
    pd = w.add(f"PRODUCT_DEFINITION('design','',{pdf},{pdc})")
    pds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{pd})")
    return pd, pds


def box_brep(w):
    a, b, c = BOX
    corners = [(x * a, y * b, z * c) for z in (0, 1) for y in (0, 1) for x in (0, 1)]
    vp = [w.add(f"VERTEX_POINT('',{w.pt(p)})") for p in corners]
    edges = {}

    def edge(i, j):
        key = (min(i, j), max(i, j))
        if key not in edges:
            p, q = corners[key[0]], corners[key[1]]
            d = tuple(qq - pp for pp, qq in zip(p, q))
            ln = math.sqrt(sum(x * x for x in d))
            line = w.add(f"LINE('',{w.pt(p)},{w.add(f'VECTOR({chr(39)}{chr(39)},{w.dir(tuple(x / ln for x in d))},{ln:.6f})')})")
            edges[key] = w.add(f"EDGE_CURVE('',{vp[key[0]]},{vp[key[1]]},{line},.T.)")
        return w.add(f"ORIENTED_EDGE('',*,*,{edges[key]},{'.T.' if i < j else '.F.'})")

    faces = []
    for axis in range(3):
        for side in (0, 1):
            n = [0, 0, 0]; n[axis] = 1 if side else -1
            idx = [k for k, p in enumerate(corners) if p[axis] == (side * BOX[axis])]
            u = [0, 0, 0]; u[(axis + 1) % 3] = 1
            v = cross(n, u)
            cen = [sum(corners[k][m] for k in idx) / 4 for m in range(3)]
            idx.sort(key=lambda k: math.atan2(sum((corners[k][m] - cen[m]) * v[m] for m in range(3)),
                                               sum((corners[k][m] - cen[m]) * u[m] for m in range(3))))
            loop = w.add("EDGE_LOOP('',(%s))" % ",".join(edge(idx[m], idx[(m + 1) % 4]) for m in range(4)))
            bound = w.add(f"FACE_OUTER_BOUND('',{loop},.T.)")
            plane = w.add(f"PLANE('',{w.ax((tuple(cen), tuple(n), tuple(u)))})")
            faces.append(w.add(f"ADVANCED_FACE('',({bound}),{plane},.T.)"))
    shell = w.add("CLOSED_SHELL('',(%s))" % ",".join(faces))
    return w.add(f"MANIFOLD_SOLID_BREP('Box',{shell})")


def make(name, levels):
    """levels: list of (parent_frame, child_frame, child_first, items_swapped) from top to the part."""
    w = Writer()
    appctx = w.add("APPLICATION_CONTEXT('automotive design')")
    w.add(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,{appctx})")
    # every product gets its own representation context, as most exporters do (a context shared by the
    # representations of several products makes the importer attach all their items to every product)
    # the part
    ctx = context(w)
    part_pd, part_pds = product(w, appctx, "Box")
    child_frame = levels[-1][1]
    part_place = w.ax(child_frame)
    part_sr = w.add(f"SHAPE_REPRESENTATION('Box',({part_place}),{ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({part_pds},{part_sr})")
    brep = box_brep(w)
    absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('Box',({brep},{w.ax(IDENT)}),{ctx})")
    w.add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{part_sr},{absr})")
    child = (part_pd, part_sr, part_place)
    # assemblies from the bottom up
    for k, (pframe, cframe, child_first, swapped) in enumerate(reversed(levels)):
        aname = "Top" if k == len(levels) - 1 else f"Sub{k}"
        apd, apds = product(w, appctx, aname)
        ctx = context(w)
        aplace = w.ax(pframe)
        # the own placement of this assembly as seen from its parent (identity for the top level)
        own = w.ax(levels[len(levels) - 2 - k][1]) if k < len(levels) - 1 else w.ax(IDENT)
        asr = w.add(f"SHAPE_REPRESENTATION('{aname}',({own},{aplace}),{ctx})")
        w.add(f"SHAPE_DEFINITION_REPRESENTATION({apds},{asr})")
        cpd, csr, cplace = child
        nauo = w.add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('{aname}-{k}','{aname}-{k}','',{apd},{cpd},$)")
        npds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{nauo})")
        rep1, rep2 = (csr, asr) if child_first else (asr, csr)
        it1, it2 = (cplace, aplace) if child_first else (aplace, cplace)
        if swapped: it1, it2 = it2, it1
        idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{it1},{it2})")
        rr = w.add(f"(REPRESENTATION_RELATIONSHIP('','',{rep1},{rep2})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt})"
                   f"SHAPE_REPRESENTATION_RELATIONSHIP())")
        w.add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{npds})")
        child = (apd, asr, own)
    with open(name, "w", newline="\n") as f:
        f.write(w.text(name))


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


def expected(chain):
    """chain: list of (parent_frame, child_frame) from top to the part; world = P0 C0^-1 P1 C1^-1 ... box"""
    a, b, c = BOX
    pts = [[x * a, y * b, z * c] for x in (0, 1) for y in (0, 1) for z in (0, 1)]
    for pf, cf in reversed(chain):
        pts = [apply_frame(pf, apply_inverse(cf, p)) for p in pts]
    lo = [min(p[r] for p in pts) for r in range(3)]
    hi = [max(p[r] for p in pts) for r in range(3)]
    return lo + hi


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
                    print(f2, [round(v, 6) for v in expected([(pf, cf)])])
    # two levels: Top -(P1, child first)-> Sub -(Q, parent first, child placement FC)-> Box
    make("box_two_levels.stp", [(P1, IDENT, True, False), (Q, FC, False, False)])
    print("box_two_levels.stp", [round(v, 6) for v in expected([(P1, IDENT), (Q, FC)])])
