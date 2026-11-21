"""Structural analyzer for STEP AP203/AP214/AP242 files (no geometry).

For each file it reports the product structure, which entities hold the shapes,
and for every assembly occurrence (CONTEXT_DEPENDENT_SHAPE_REPRESENTATION) how the
representation relationship is oriented and which of CADability's two
transformation formulas (transformationMode false/true) yields the correct placement.
"""
import re, sys, json, math, os, traceback
from collections import defaultdict, Counter

STMT = re.compile(r"#(\d+)\s*=\s*((?:'(?:[^']|'')*'|[^;'])*);", re.S)
COMMENT = re.compile(r"/\*.*?\*/", re.S)
TOK = re.compile(r"\s*(?:(#\d+)|('(?:[^']|'')*')|(\.[A-Z0-9_]+\.)|([-+]?(?:\d+\.?\d*|\.\d+)(?:[EeDd][-+]?\d+)?)|([A-Z_][A-Z0-9_]*)|(\$)|(\*)|(\()|(\))|(,))")

class Ref(int):
    pass

def parse_args(s, pos=0):
    """Parse a parenthesized argument list starting at s[pos]=='('; returns (list, newpos)."""
    out = []
    m = TOK.match(s, pos)
    assert m and m.group(8), s[pos:pos + 40]
    pos = m.end()
    while True:
        m = TOK.match(s, pos)
        if not m:
            raise ValueError("bad token at " + s[pos:pos + 40])
        if m.group(9):
            return out, m.end()
        if m.group(10):
            pos = m.end(); continue
        if m.group(1): out.append(Ref(int(m.group(1)[1:]))); pos = m.end()
        elif m.group(2): out.append(m.group(2)[1:-1].replace("''", "'")); pos = m.end()
        elif m.group(3): out.append(m.group(3)); pos = m.end()
        elif m.group(4): out.append(float(m.group(4).replace('D', 'E').replace('d', 'e'))); pos = m.end()
        elif m.group(5):  # typed parameter like LENGTH_MEASURE(1.0)
            name = m.group(5)
            sub, pos = parse_args(s, m.end())
            out.append((name, sub))
        elif m.group(6) or m.group(7): out.append(None); pos = m.end()
        elif m.group(8):
            sub, pos = parse_args(s, m.start())
            out.append(sub)

def parse_body(body):
    """Return dict type -> args (complex entities have several types)."""
    body = body.strip()
    res = {}
    if body.startswith('('):  # complex instance
        pos = 1
        while True:
            m = re.compile(r"\s*([A-Z_][A-Z0-9_]*)\s*").match(body, pos)
            if not m: break
            args, pos = parse_args(body, m.end())
            res[m.group(1)] = args
    else:
        m = re.match(r"([A-Z_][A-Z0-9_]*)\s*", body)
        args, _ = parse_args(body, m.end())
        res[m.group(1)] = args
    return res

TYPE_RE = re.compile(r"\s*\(?\s*([A-Z_][A-Z0-9_]*)")
TYPES_IN_COMPLEX = re.compile(r"([A-Z_][A-Z0-9_]*)\s*\(")

class StepFile:
    def __init__(self, path):
        with open(path, 'rb') as f:
            raw = f.read()
        txt = raw.decode('latin-1')
        hdr_end = txt.find('DATA;')
        self.header = txt[:hdr_end] if hdr_end >= 0 else ''
        data = COMMENT.sub('', txt[hdr_end:]) if hdr_end >= 0 else ''
        self.body = {}
        self.types = {}  # id -> tuple of type names
        for m in STMT.finditer(data):
            i = int(m.group(1)); b = m.group(2)
            self.body[i] = b
            bs = b.lstrip()
            if bs.startswith('('):
                # names of the partial entities on the outer level
                self.types[i] = tuple(self._outer_types(bs))
            else:
                self.types[i] = (TYPE_RE.match(bs).group(1),)
        self._cache = {}

    @staticmethod
    def _outer_types(bs):
        names = []; depth = 0; i = 0; n = len(bs)
        while i < n:
            c = bs[i]
            if c == "'":
                i += 1
                while i < n:
                    if bs[i] == "'":
                        if i + 1 < n and bs[i + 1] == "'": i += 2; continue
                        break
                    i += 1
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
            elif depth == 1 and (c.isalpha() or c == '_'):
                j = i
                while j < n and (bs[j].isalnum() or bs[j] == '_'): j += 1
                names.append(bs[i:j]); i = j; continue
            i += 1
        return names

    def ent(self, i):
        e = self._cache.get(i)
        if e is None:
            e = parse_body(self.body[i]); self._cache[i] = e
        return e

    def t(self, i):
        return self.types.get(i, ('?',))

    def first(self, i, *names):
        e = self.ent(i)
        for n in names:
            if n in e: return e[n]
        return None

    def ids_of(self, name):
        return [i for i, ts in self.types.items() if name in ts]

    def ids_like(self, pred):
        return [i for i, ts in self.types.items() if any(pred(x) for x in ts)]

# ---------------------------------------------------------------- geometry helpers

def vnorm(v):
    l = math.sqrt(sum(x * x for x in v)); return [x / l for x in v] if l > 0 else v
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def matvec(M, v): return [dot(M[r], v) for r in range(3)]
def matmul(A, B): return [[sum(A[r][k] * B[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
def transpose(A): return [[A[c][r] for c in range(3)] for r in range(3)]
def add(a, b): return [x + y for x, y in zip(a, b)]
def sub(a, b): return [x - y for x, y in zip(a, b)]
def scale(a, s): return [x * s for x in a]

class Frame:
    def __init__(self, loc, x, y, z):
        self.loc, self.x, self.y, self.z = loc, x, y, z
    def R(self):  # columns are the axes
        return [[self.x[r], self.y[r], self.z[r]] for r in range(3)]
    def is_identity(self, tol=1e-9):
        return (max(abs(v) for v in self.loc) < tol and abs(self.x[0] - 1) < tol and abs(self.y[1] - 1) < tol and abs(self.z[2] - 1) < tol)

def transform_eq(A, B, size):
    (Ra, ta), (Rb, tb) = A, B
    dr = max(abs(Ra[r][c] - Rb[r][c]) for r in range(3) for c in range(3))
    dt = max(abs(x - y) for x, y in zip(ta, tb))
    return dr < 1e-6 and dt < 1e-6 * max(1.0, size)

# ---------------------------------------------------------------- analysis

def analyze(path):
    sf = StepFile(path)
    r = {'file': path, 'size': os.path.getsize(path)}
    m = re.search(r"FILE_SCHEMA\s*\(\s*\(\s*'([^']*)'", sf.header)
    r['schema'] = m.group(1) if m else None
    m = re.search(r"FILE_NAME\s*\((.*?)\)\s*;", sf.header, re.S)
    if m:
        try:
            fn = parse_args('(' + m.group(1) + ')')[0]
            r['system'] = (fn[5] or '') + ' | ' + (fn[4] or '')
        except Exception:
            r['system'] = None

    tc = Counter(t for ts in sf.types.values() for t in ts)
    keys = ['PRODUCT', 'PRODUCT_DEFINITION', 'NEXT_ASSEMBLY_USAGE_OCCURRENCE', 'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION',
            'SHAPE_DEFINITION_REPRESENTATION', 'SHAPE_REPRESENTATION_RELATIONSHIP', 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION',
            'ITEM_DEFINED_TRANSFORMATION', 'CARTESIAN_TRANSFORMATION_OPERATOR_3D', 'MAPPED_ITEM', 'REPRESENTATION_MAP',
            'MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION', 'DRAUGHTING_MODEL', 'STYLED_ITEM', 'OVER_RIDING_STYLED_ITEM',
            'PRESENTATION_LAYER_ASSIGNMENT', 'MANIFOLD_SOLID_BREP', 'BREP_WITH_VOIDS', 'SHELL_BASED_SURFACE_MODEL', 'FACETED_BREP',
            'GEOMETRIC_SET', 'GEOMETRIC_CURVE_SET', 'ADVANCED_FACE', 'SHAPE_ASPECT', 'CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP',
            'TESSELLATED_SHELL', 'TESSELLATED_SOLID', 'ASSEMBLY_COMPONENT_USAGE', 'SPECIFIED_HIGHER_USAGE_OCCURRENCE']
    r['counts'] = {k: tc[k] for k in keys if tc[k]}

    # representations: types ending with REPRESENTATION (excluding relationships/definition reps)
    def is_rep_type(n):
        return n.endswith('REPRESENTATION') and n not in ('SHAPE_DEFINITION_REPRESENTATION', 'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION') or n == 'DRAUGHTING_MODEL'
    reps = {}
    for i, ts in sf.types.items():
        if any(is_rep_type(x) for x in ts):
            e = sf.ent(i)
            args = None
            for n in ('REPRESENTATION',) + ts:
                if n in e and isinstance(e[n], list) and len(e[n]) >= 3:
                    args = e[n]; break
            if args is not None:
                reps[i] = {'name': args[0], 'items': [x for x in (args[1] or []) if isinstance(x, Ref)], 'ctx': args[2], 'type': ts}

    # length unit factor per context (to mm)
    ctxfac = {}
    def unit_factor(u):
        e = sf.ent(u)
        if 'SI_UNIT' in e and 'LENGTH_UNIT' in e:
            pre = e['SI_UNIT'][0] if len(e['SI_UNIT']) == 2 else e['SI_UNIT'][-2]
            return {'.MILLI.': 1.0, '.CENTI.': 10.0, '.DECI.': 100.0, '.KILO.': 1e6, '.MICRO.': 1e-3, None: 1000.0}.get(pre, 1000.0)
        if 'CONVERSION_BASED_UNIT' in e and 'LENGTH_UNIT' in e:
            nm, meas = e['CONVERSION_BASED_UNIT'][0], e['CONVERSION_BASED_UNIT'][1]
            me = sf.ent(meas)
            for k, a in me.items():
                if k.endswith('MEASURE_WITH_UNIT'):
                    val = a[0][1][0] if isinstance(a[0], tuple) else a[0]
                    return val * unit_factor(a[1])
        return None
    def ctx_factor(c):
        if c in ctxfac: return ctxfac[c]
        f = None
        try:
            e = sf.ent(c)
            if 'GLOBAL_UNIT_ASSIGNED_CONTEXT' in e:
                for u in e['GLOBAL_UNIT_ASSIGNED_CONTEXT'][0]:
                    if 'LENGTH_UNIT' in sf.t(u):
                        f = unit_factor(u)
        except Exception:
            f = None
        ctxfac[c] = f
        return f

    def point(i):
        a = sf.first(i, 'CARTESIAN_POINT'); return list(a[1]) + [0.0] * (3 - len(a[1]))
    def direction(i):
        a = sf.first(i, 'DIRECTION'); return vnorm(list(a[1]) + [0.0] * (3 - len(a[1])))
    def frame(i, fac):
        a = sf.first(i, 'AXIS2_PLACEMENT_3D')
        if a is None: return None
        loc = scale(point(a[1]), fac or 1.0)
        z = direction(a[2]) if a[2] is not None else [0.0, 0.0, 1.0]
        x = direction(a[3]) if a[3] is not None else ([1.0, 0, 0] if abs(z[0]) < 0.9 else [0, 1.0, 0])
        x = vnorm(sub(x, scale(z, dot(x, z))))
        y = cross(z, x)
        return Frame(loc, x, y, z)

    # product structure
    pd_product = {}
    for i in sf.ids_like(lambda n: n.startswith('PRODUCT_DEFINITION') and n not in ('PRODUCT_DEFINITION_SHAPE', 'PRODUCT_DEFINITION_CONTEXT', 'PRODUCT_DEFINITION_FORMATION', 'PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE', 'PRODUCT_DEFINITION_RELATIONSHIP', 'PRODUCT_DEFINITION_USAGE')):
        a = sf.first(i, *sf.t(i))
        try:
            form = a[2]
            prod = sf.first(form, *sf.t(form))[2]
            pd_product[i] = prod
        except Exception:
            pass
    prod_pds = defaultdict(list)
    for pd, p in pd_product.items(): prod_pds[p].append(pd)
    r['products_with_several_pds'] = sum(1 for v in prod_pds.values() if len(v) > 1)

    def prod_name(pd):
        p = pd_product.get(pd)
        if p is None: return '?'
        a = sf.first(p, 'PRODUCT'); return a[0] if a else '?'

    nauo = {}
    for i in sf.ids_of('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
        a = sf.first(i, 'NEXT_ASSEMBLY_USAGE_OCCURRENCE')
        nauo[i] = (a[3], a[4])  # relating (parent), related (child)
    children = set(c for _, c in nauo.values())
    parents = set(p for p, _ in nauo.values())
    root_pds = [pd for pd in pd_product if pd not in children]
    r['root_pds'] = [(pd, prod_name(pd)) for pd in root_pds][:10]
    r['n_root_pds'] = len(root_pds)

    # PDS -> PD (or shape_aspect)
    def pds_target(pds):
        a = sf.first(pds, 'PRODUCT_DEFINITION_SHAPE', 'PROPERTY_DEFINITION')
        if a is None: return None
        d = a[2]
        if 'SHAPE_ASPECT' in sf.t(d):
            ofs = sf.first(d, 'SHAPE_ASPECT')[2]
            return pds_target(ofs)
        return d

    # SDR: PD -> reps
    pd_reps = defaultdict(set)
    sdr_targets = Counter()
    for i in sf.ids_of('SHAPE_DEFINITION_REPRESENTATION'):
        a = sf.first(i, 'SHAPE_DEFINITION_REPRESENTATION')
        tgt = pds_target(a[0])
        sdr_targets[sf.t(tgt)[0] if tgt else 'None'] += 1
        if tgt is not None: pd_reps[tgt].add(a[1])
    r['sdr_targets'] = dict(sdr_targets)

    # plain SRR (no transformation): rep equivalences
    rr_plain = []
    rr_trans = {}
    for i, ts in sf.types.items():
        if 'REPRESENTATION_RELATIONSHIP' in ts or 'SHAPE_REPRESENTATION_RELATIONSHIP' in ts or 'CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP' in ts:
            e = sf.ent(i)
            base = e.get('REPRESENTATION_RELATIONSHIP') or e.get('SHAPE_REPRESENTATION_RELATIONSHIP') or e.get('CONSTRUCTIVE_GEOMETRY_REPRESENTATION_RELATIONSHIP')
            if base is None or len(base) < 4: continue
            if 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION' in e:
                rr_trans[i] = (base[2], base[3], e['REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION'][0])
            else:
                rr_plain.append((i, base[2], base[3]))
    eq = defaultdict(set)
    for _, a, b in rr_plain:
        eq[a].add(b); eq[b].add(a)
    def closure(reps0):
        seen = set(reps0); st = list(reps0)
        while st:
            x = st.pop()
            for y in eq.get(x, ()):
                if y not in seen: seen.add(y); st.append(y)
        return seen
    pd_allreps = {pd: closure(rs) for pd, rs in pd_reps.items()}

    geo_types = {'MANIFOLD_SOLID_BREP', 'BREP_WITH_VOIDS', 'SHELL_BASED_SURFACE_MODEL', 'FACETED_BREP', 'GEOMETRIC_SET', 'GEOMETRIC_CURVE_SET',
                 'ADVANCED_FACE', 'TESSELLATED_SHELL', 'TESSELLATED_SOLID', 'MAPPED_ITEM', 'OPEN_SHELL', 'CLOSED_SHELL', 'FACE_SURFACE'}
    def rep_geo(rep):
        rp = reps.get(rep)
        if not rp: return []
        return [it for it in rp['items'] if any(t in geo_types for t in sf.t(it))]

    # which geometry items are reachable from the product structure?
    reach_geo = set()
    for pd, rs in pd_allreps.items():
        for rp in rs: reach_geo.update(rep_geo(rp))
    # follow mapped items
    st = [g for g in reach_geo if 'MAPPED_ITEM' in sf.t(g)]
    while st:
        mi = st.pop()
        a = sf.first(mi, 'MAPPED_ITEM'); rmap = sf.first(a[1], 'REPRESENTATION_MAP')
        for rp in closure({rmap[1]}):
            for g in rep_geo(rp):
                if g not in reach_geo:
                    reach_geo.add(g)
                    if 'MAPPED_ITEM' in sf.t(g): st.append(g)
    # also geometry reached through rr_trans reps of known PDs is already included via closure? (rr_trans are not equivalences)
    top_geo = [i for i, ts in sf.types.items() if any(t in ('MANIFOLD_SOLID_BREP', 'BREP_WITH_VOIDS', 'SHELL_BASED_SURFACE_MODEL', 'FACETED_BREP', 'TESSELLATED_SOLID', 'TESSELLATED_SHELL') for t in ts)]
    r['solids_total'] = len(top_geo)
    r['solids_unreached_from_products'] = len([g for g in top_geo if g not in reach_geo])
    # where are the unreached ones referenced from?
    if r['solids_unreached_from_products']:
        unreached = set(g for g in top_geo if g not in reach_geo)
        users = Counter()
        for i, b in sf.body.items():
            for mm in re.finditer(r"#(\d+)", b):
                if int(mm.group(1)) in unreached:
                    users[sf.t(i)[0] if len(sf.t(i)) == 1 else '+'.join(sf.t(i))] += 1
        r['unreached_used_by'] = dict(users.most_common(6))

    # mapped items in reps directly owned by PDs
    r['pd_reps_with_mapped_items'] = sum(1 for pd, rs in pd_allreps.items() if any('MAPPED_ITEM' in sf.t(g) for rp in rs for g in rep_geo(rp)))

    # ---------------------------------------------------------------- occurrences
    occ = []
    orient = Counter(); verdict = Counter(); member = Counter(); idents = Counter(); unitmix = 0
    for i in sf.ids_of('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
        a = sf.first(i, 'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION')
        rr, pds = a[0], a[1]
        pa = sf.first(pds, 'PRODUCT_DEFINITION_SHAPE', 'PROPERTY_DEFINITION')
        n = pa[2] if pa else None
        if n not in nauo:
            orient['cdsr_without_nauo'] += 1; continue
        parent, child = nauo[n]
        if rr not in rr_trans:
            orient['rr_without_transformation'] += 1; continue
        rep1, rep2, op = rr_trans[rr]
        preps = pd_allreps.get(parent, set()); creps = pd_allreps.get(child, set())
        if rep1 in creps and rep2 in preps: o = 'rep1=child'
        elif rep1 in preps and rep2 in creps: o = 'rep1=parent'
        else: o = 'undetermined'
        orient[o] += 1
        if 'ITEM_DEFINED_TRANSFORMATION' not in sf.t(op):
            verdict['op:' + sf.t(op)[0]] += 1; continue
        ia = sf.first(op, 'ITEM_DEFINED_TRANSFORMATION')
        it1, it2 = ia[2], ia[3]
        r1items = reps[rep1]['items'] if rep1 in reps else []
        r2items = reps[rep2]['items'] if rep2 in reps else []
        member[('it1@rep1' if it1 in r1items else 'it1@rep2' if it1 in r2items else 'it1@?') + ',' + ('it2@rep2' if it2 in r2items else 'it2@rep1' if it2 in r1items else 'it2@?')] += 1
        f1 = ctx_factor(reps[rep1]['ctx']) if rep1 in reps else 1.0
        f2 = ctx_factor(reps[rep2]['ctx']) if rep2 in reps else 1.0
        if (f1 or 1.0) != (f2 or 1.0): unitmix += 1
        # ---- the correct placement: child frame -> parent frame
        # which item belongs to the child representation?
        cset = creps; pset = preps
        def owner(it):
            for rp in (rep1, rep2):
                if rp in reps and it in reps[rp]['items']:
                    return rp
            return None
        o1, o2 = owner(it1), owner(it2)
        if o == 'undetermined' or o1 is None or o2 is None or o1 == o2:
            verdict['correct_unknown'] += 1; continue
        c_it, p_it = (it1, it2) if o1 in cset else (it2, it1)
        c_rep, p_rep = (o1, o2) if o1 in cset else (o2, o1)
        C = frame(c_it, ctx_factor(reps[c_rep]['ctx'])); P = frame(p_it, ctx_factor(reps[p_rep]['ctx']))
        if C is None or P is None:
            verdict['no_axis2placement'] += 1; continue
        Rc, Rp = C.R(), P.R()
        Rm = matmul(Rp, transpose(Rc))
        tm = sub(P.loc, matvec(Rm, C.loc))
        correct = (Rm, tm)
        idents[('C=id' if C.is_identity() else 'C!=id') + ',' + ('P=id' if P.is_identity() else 'P!=id')] += 1
        # ---- CADability emulation (GetTransformation)
        org = frame(it1, ctx_factor(reps[rep1]['ctx'])); trg = frame(it2, ctx_factor(reps[rep2]['ctx']))
        code1 = code2 = 0
        for x in r1items:
            if x == it1: code1 = 1
            elif x == it2: code1 = -1
            if code1: break
        for x in r2items:
            if x == it1: code2 = -1
            elif x == it2: code2 = 1
            if code2: break
        if code1 != 1 and code2 != 1 and code1 == -1 and code2 == -1:
            org, trg = trg, org
        Ro, Rt = org.R(), trg.R()
        # formula for transformationMode == false (note: MA1.Transpose() is a no-op in MathNet)
        F1 = (matmul(Ro, Rt), matvec(transpose(Ro), sub(trg.loc, org.loc)))
        # transformationMode == true: Translate(org-trg) * Fit(trg axes -> org axes)
        F2 = (matmul(Ro, transpose(Rt)), sub(org.loc, trg.loc))
        size = max(1.0, max(abs(v) for v in P.loc + C.loc))
        ok1 = transform_eq(F1, correct, size); ok2 = transform_eq(F2, correct, size)
        inv = (transpose(Rm), scale(matvec(transpose(Rm), tm), -1.0))
        k = ('false_ok' if ok1 else 'false_bad') + '|' + ('true_ok' if ok2 else 'true_bad')
        if not ok1 and not ok2 and (transform_eq(F1, inv, size) or transform_eq(F2, inv, size)):
            k += '|one_is_inverse'
        verdict[k] += 1
        verdict[o + ':' + k] += 1
        if len(occ) < 3:
            occ.append({'cdsr': i, 'parent': prod_name(parent), 'child': prod_name(child), 'orient': o, 'C': C.loc, 'P': P.loc, 'res': k})
    r['orientation'] = dict(orient)
    r['placement_membership'] = dict(member)
    r['identity'] = dict(idents)
    r['verdict'] = dict(verdict)
    r['unit_mix'] = unitmix
    r['examples'] = occ
    # summary of which mode works for the whole file
    n_ok_f = sum(v for k, v in verdict.items() if ':' not in k and k.startswith('false_ok'))
    n_ok_t = sum(v for k, v in verdict.items() if ':' not in k and '|true_ok' in k)
    n_tot = sum(v for k, v in verdict.items() if ':' not in k and k.startswith('false_'))
    r['mode_summary'] = {'occurrences': n_tot, 'false_ok': n_ok_f, 'true_ok': n_ok_t}
    # MDGPR / styled items covering
    return r

def main():
    out = sys.argv[2]
    files = [l.strip() for l in open(sys.argv[1], encoding='utf-8-sig') if l.strip()]
    from concurrent.futures import ProcessPoolExecutor, as_completed
    with open(out, 'w', encoding='utf-8') as fo, ProcessPoolExecutor(max_workers=int(sys.argv[3]) if len(sys.argv) > 3 else 6) as ex:
        futs = {ex.submit(safe, f): f for f in files}
        for k, fu in enumerate(as_completed(futs)):
            fo.write(json.dumps(fu.result(), default=str) + '\n'); fo.flush()

def safe(f):
    try:
        return analyze(f)
    except Exception as e:
        return {'file': f, 'error': repr(e), 'trace': traceback.format_exc()[-800:]}

if __name__ == '__main__':
    if len(sys.argv) == 2:
        print(json.dumps(analyze(sys.argv[1]), indent=1, default=str))
    else:
        main()
