"""Compare the geometric results of two corpus runs of StepProbe (see runcorpus.py).

usage: geomcompare.py <dir A> <label A> <dir B> <label B> [--noise <dir A2> <label A2>]
A is the baseline (old state), B the changed state. The files are taken from <dir A>/index.tsv.
For every file the status, the number of problems reported by the import, the solids and shells, their open
edges, faces, volumes and surface types are compared, and each changed file gets a verdict:
  worse    only deteriorations (exception/timeout, more problems, fewer solids, more open edges, inward solids, ...)
  better   only improvements
  mixed    both
  changed  only neutral changes (other faces, surface types or volumes)
Some imports are not deterministic (the solids are created in parallel). With --noise, a second run of the
baseline (A2) is compared to A: files that differ there are reported separately as noisy, their differences
between A and B are not reliable.
At the end the surface types of all faces are summed up, which shows e.g. how many NURBS surfaces have been
replaced by analytic surfaces.
"""
import os, re, sys
from collections import Counter

VOLUME_TOLERANCE = 2e-3  # relative, the volume is computed from a triangulation with 1e-4 of the size

BODY = re.compile(r"^# body (\w+) '(.*)' faces (-?\d+) edges (-?\d+) open (-?\d+) volume (\S+) surfaces (.*)$")


def parse(path):
    r = {'status': 'ok', 'time': None, 'top': 0, 'problems': 0, 'bodies': [], 'debug': False}
    if not os.path.exists(path):
        r['status'] = 'MISSING'
        return r
    for l in open(path, encoding='utf-8-sig', errors='replace').read().splitlines():
        if l.startswith('# cadability ') and l.endswith('DEBUG'): r['debug'] = True
        elif l.startswith('# EXCEPTION'): r['status'] = 'EXCEPTION'
        elif l.startswith('# CRASH'): r['status'] = 'CRASH'
        elif l.startswith('# TIMEOUT'): r['status'] = 'TIMEOUT'
        elif l.startswith('# time '):
            m = re.match(r'# time ([\d.,]+)s, top level (\d+)', l)
            if m: r['time'], r['top'] = float(m.group(1).replace(',', '.')), int(m.group(2))
        elif l.startswith('# problems '): r['problems'] = int(l.split()[2])
        else:
            m = BODY.match(l)
            if m:
                try: volume = float(m.group(6))
                except ValueError: volume = None
                surfaces = Counter()
                for item in m.group(7).split(','):
                    if ':' in item:
                        k, v = item.rsplit(':', 1)
                        surfaces[k] += int(v)
                r['bodies'].append({'type': m.group(1), 'faces': int(m.group(3)), 'edges': int(m.group(4)),
                                    'open': int(m.group(5)), 'volume': volume, 'surfaces': surfaces})
    return r


def metrics(r):
    solids = [b for b in r['bodies'] if b['type'] == 'Solid']
    shells = [b for b in r['bodies'] if b['type'] == 'Shell']
    surfaces = Counter()
    for b in r['bodies']: surfaces.update(b['surfaces'])
    return {
        'status': r['status'], 'top': r['top'], 'problems': r['problems'],
        'solids': len(solids), 'shells': len(shells),
        'open': sum(max(b['open'], 0) for b in r['bodies']),
        'faces': sum(b['faces'] for b in r['bodies']),
        'inward': sum(1 for b in solids if b['volume'] is not None and b['volume'] < 0),
        'volumes': sorted(b['volume'] for b in solids if b['volume'] is not None),
        'surfaces': surfaces,
    }


def same_volumes(va, vb):
    if len(va) != len(vb): return False
    for x, y in zip(va, vb):
        if abs(x - y) > VOLUME_TOLERANCE * max(abs(x), abs(y), 1e-9): return False
    return True


def diff(a, b):
    """Returns (worse, better, neutral): lists of short descriptions of the differences from a to b."""
    worse, better, neutral = [], [], []
    ma, mb = metrics(a), metrics(b)
    if ma['status'] != mb['status']:
        (worse if ma['status'] == 'ok' else better if mb['status'] == 'ok' else neutral).append(f"status {ma['status']}->{mb['status']}")
    if ma['status'] != 'ok' or mb['status'] != 'ok': return worse, better, neutral

    def compare(name, higher_is_better):
        x, y = ma[name], mb[name]
        if x == y: return
        ((better if (y > x) == higher_is_better else worse)).append(f'{name} {x}->{y}')
    compare('top', True)
    compare('problems', False)
    compare('solids', True)
    if ma['solids'] + ma['shells'] == mb['solids'] + mb['shells']:
        if ma['shells'] != mb['shells']: neutral.append(f"shells {ma['shells']}->{mb['shells']}")
    else: compare('shells', True)  # bodies lost or gained
    compare('open', False)
    compare('inward', False)
    if ma['faces'] != mb['faces']: neutral.append(f"faces {ma['faces']}->{mb['faces']}")
    if ma['solids'] == mb['solids'] and not same_volumes(ma['volumes'], mb['volumes']): neutral.append('volume')
    if ma['surfaces'] != mb['surfaces']:
        changes = []
        for k in sorted(set(ma['surfaces']) | set(mb['surfaces'])):
            d = mb['surfaces'][k] - ma['surfaces'][k]
            if d: changes.append(f'{k.replace("Surface", "")}{d:+d}')
        neutral.append('surfaces ' + ' '.join(changes))
    return worse, better, neutral


def main():
    args = sys.argv[1:]
    noise = None
    if '--noise' in args:
        k = args.index('--noise')
        noise = (args[k + 1], args[k + 2])
        del args[k:k + 3]
    if len(args) != 4:
        print(__doc__)
        sys.exit(1)
    dir_a, label_a, dir_b, label_b = args
    index = [l.rstrip('\n').split('\t') for l in open(os.path.join(dir_a, 'index.tsv'), encoding='utf-8')]
    sections = {'worse': [], 'mixed': [], 'better': [], 'changed': [], 'noisy': []}
    surf_a, surf_b = Counter(), Counter()
    time_a = time_b = 0.0
    slower = []
    unchanged = 0
    debug_runs = set()
    for n, path in index:
        a = parse(os.path.join(dir_a, f'{n}.{label_a}.txt'))
        b = parse(os.path.join(dir_b, f'{n}.{label_b}.txt'))
        if a['debug']: debug_runs.add(label_a)
        if b['debug']: debug_runs.add(label_b)
        surf_a.update(metrics(a)['surfaces'])
        surf_b.update(metrics(b)['surfaces'])
        if a['time'] is not None and b['time'] is not None:
            time_a += a['time']
            time_b += b['time']
            if b['time'] > 2 * a['time'] and b['time'] - a['time'] > 10: slower.append(f'{n} {path}: {a["time"]:.0f}s -> {b["time"]:.0f}s')
        worse, better, neutral = diff(a, b)
        if not (worse or better or neutral):
            unchanged += 1
            continue
        line = f'{n} {path}: ' + ', '.join([*('-' + w for w in worse), *('+' + x for x in better), *neutral])
        if noise:
            a2 = parse(os.path.join(noise[0], f'{n}.{noise[1]}.txt'))
            nw, nb, nn = diff(a, a2)
            if nw or nb or nn:
                sections['noisy'].append(line + '   [baseline itself: ' + ', '.join(nw + nb + nn) + ']')
                continue
        if worse and better: sections['mixed'].append(line)
        elif worse: sections['worse'].append(line)
        elif better: sections['better'].append(line)
        else: sections['changed'].append(line)
    if debug_runs: print(f'WARNING: a Debug build of CADability was used for {", ".join(sorted(debug_runs))}, the comparison is not meaningful')
    print(f'{len(index)} files: {unchanged} unchanged, ' + ', '.join(f'{len(v)} {k}' for k, v in sections.items()))
    print(f'import time (files without timeout in both runs): {time_a / 60:.1f} min -> {time_b / 60:.1f} min')
    explanation = {
        'worse': 'only deteriorations',
        'mixed': 'deteriorations and improvements',
        'better': 'only improvements',
        'changed': 'only neutral changes',
        'noisy': 'the baseline differs from its second run, the differences are not reliable',
    }
    for k, v in sections.items():
        if v:
            print(f'\n== {k} ({len(v)}): {explanation[k]}')
            for line in v: print('  ' + line)
    if slower:
        print(f'\n== slower ({len(slower)}): more than twice and more than 10 s slower')
        for line in slower: print('  ' + line)
    print('\n== surface types of all faces')
    for k in sorted(set(surf_a) | set(surf_b)):
        print(f'  {k:28} {surf_a[k]:8d} -> {surf_b[k]:8d}  {surf_b[k] - surf_a[k]:+d}')


if __name__ == '__main__':
    main()
