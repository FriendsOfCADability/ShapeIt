"""Compare two corpus runs of StepProbe (see runcorpus.py).

usage: compare.py <dir A> <label A> <dir B> <label B> [--orientation <file.tsv>]
Compares <dir A>/<n>.<label A>.txt with <dir B>/<n>.<label B>.txt for every file of <dir A>/index.tsv.
Children of a block are compared independent of their order (the import creates them in parallel),
extents with a tolerance of 2e-5 * size (the dumps have 6 significant digits).
With --orientation (file<TAB>orientation lines, as written by list_assemblies.py) the orientation
is printed next to each reported file.
Note: some imports are not deterministic (parallel creation, global unit factor), so a reported
difference should be confirmed by importing the file again with both versions.
"""
import sys, os, re

args = sys.argv[1:]
orientation = {}
if '--orientation' in args:
    k = args.index('--orientation')
    orientation = dict(l.rstrip('\n').split('\t') for l in open(args[k + 1], encoding='utf-8'))
    del args[k:k + 2]
dir_a, label_a, dir_b, label_b = args
index = [l.rstrip('\n').split('\t') for l in open(os.path.join(dir_a, 'index.tsv'), encoding='utf-8')]

LINE = re.compile(r"^( *)(\w+) '(.*)' (\[.*\]|empty)$")
NUMBER = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[Ee][-+]?\d+)?")


def parse(path):
    """Returns the dumped object tree (type, name, extent, children) and the error status of a dump."""
    if not os.path.exists(path): return None, 'missing'
    lines = open(path, encoding='utf-8-sig').read().splitlines()
    status = None
    for l in lines:
        if l.startswith('# EXCEPTION') or l.startswith('# CRASH') or l.startswith('# TIMEOUT'): status = l
    root = ('root', '', None, [])
    stack = [(-1, root)]
    for l in lines:
        m = LINE.match(l)
        if not m: continue
        ind = len(m.group(1))
        ext = None if m.group(4) == 'empty' else [float(x) for x in NUMBER.findall(m.group(4))]
        node = (m.group(2), m.group(3), ext, [])
        while stack[-1][0] >= ind: stack.pop()
        stack[-1][1][3].append(node)
        stack.append((ind, node))
    return root, status


def size_of(node):
    exts = []

    def walk(n):
        if n[2]: exts.append(n[2])
        for c in n[3]: walk(c)
    walk(node)
    if not exts: return 1.0
    return max(max(abs(v) for v in e) for e in exts) or 1.0


def key(n, tol):
    q = lambda v: round(v / tol) if tol else v
    return (n[0], n[1], tuple(q(v) for v in n[2]) if n[2] else (), len(n[3]))


def equal(a, b, tol, path=''):
    """None if both trees are equal, otherwise a description of the first difference."""
    if a[0] != b[0] or a[1] != b[1]: return f"{path}: {a[0]} '{a[1]}' vs {b[0]} '{b[1]}'"
    if (a[2] is None) != (b[2] is None): return f"{path}/{a[1]}: extent empty vs not"
    if a[2] and max(abs(x - y) for x, y in zip(a[2], b[2])) > tol: return f"{path}/{a[1]}: extent {a[2]} vs {b[2]}"
    if len(a[3]) != len(b[3]): return f"{path}/{a[1]}: {len(a[3])} vs {len(b[3])} children"
    ca = sorted(a[3], key=lambda n: key(n, tol * 10))
    unmatched = sorted(b[3], key=lambda n: key(n, tol * 10))
    for x in ca:  # greedy matching with tolerance
        for i, y in enumerate(unmatched):
            if equal(x, y, tol, path + '/' + a[1]) is None:
                del unmatched[i]
                break
        else:
            # report the difference to the closest candidate of the same name
            cand = [y for y in unmatched if y[0] == x[0] and y[1] == x[1]] or unmatched
            return equal(x, cand[0], tol, path + '/' + a[1]) if cand else f"{path}/{a[1]}: no match for {x[1]}"
    return None


counts = {}
for n, fn in index:
    exp, es = parse(os.path.join(dir_a, f'{n}.{label_a}.txt'))
    act, as_ = parse(os.path.join(dir_b, f'{n}.{label_b}.txt'))
    detail = ''
    if exp is None or act is None:
        res = 'missing'
    elif es or as_:
        res = 'status' if es != as_ else 'same-status'
        detail = f'{es} | {as_}'
    else:
        d = equal(exp, act, 2e-5 * size_of(exp) + 1e-9)
        res = 'equal' if d is None else 'DIFF'
        detail = d
    counts[res] = counts.get(res, 0) + 1
    if res != 'equal':
        print(f"{res:12s} {orientation.get(fn, ''):12s} {fn}\n             {detail[:300]}")
print(counts)
