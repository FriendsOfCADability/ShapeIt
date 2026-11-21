"""Summarize the results of stepstruct.py over a corpus.

usage: summarize.py <struct.jsonl>
Prints which of the two former transformation formulas fits which files, the rep_1/rep_2 orientation
per originating system, and statistics about roots, mapped items, units and representations.
"""
import json, sys
from collections import Counter, defaultdict
rows = [json.loads(l) for l in open(sys.argv[1], encoding='utf-8')]
err = [r for r in rows if 'error' in r]
ok = [r for r in rows if 'error' not in r]
print('files', len(rows), 'errors', len(err))
for r in err[:5]: print('  ERR', r['file'], r['error'][:150])

def syskey(r):
    s = (r.get('system') or '').split('|')[0].strip()
    return s[:40] or '?'

asm = [r for r in ok if r['mode_summary']['occurrences'] > 0]
print('\nfiles with assembly occurrences (CDSR+IDT):', len(asm), ' without:', len(ok) - len(asm))
cls = Counter()
bysys = defaultdict(Counter)
for r in asm:
    m = r['mode_summary']; n = m['occurrences']
    k = ('false' if m['false_ok'] == n else 'false-partial' if m['false_ok'] else 'false-no') + ' / ' + ('true' if m['true_ok'] == n else 'true-partial' if m['true_ok'] else 'true-no')
    cls[k] += 1
    o = '+'.join(sorted(r['orientation'].keys()))
    bysys[o][k] += 1
print('\nwhich mode places ALL occurrences correctly:')
for k, v in cls.most_common(): print(f'  {v:5d}  {k}')
print('\norientation vs. result:')
for o, c in bysys.items(): print(' ', o, dict(c))

print('\norientation by originating system:')
so = defaultdict(Counter)
for r in asm:
    for o, v in r['orientation'].items(): so[syskey(r)][o] += 1
for s, c in sorted(so.items(), key=lambda x: -sum(x[1].values())): print(f'  {s:40s} {dict(c)}')

print('\nplacement membership:', sum((Counter(r['placement_membership']) for r in asm), Counter()))
print('identity pattern:', sum((Counter(r['identity']) for r in asm), Counter()))
print('verdict keys:', sum((Counter({k: v for k, v in r['verdict'].items() if ':' not in k}) for r in asm), Counter()))
print('unit mix files:', sum(1 for r in asm if r['unit_mix']))

print('\nfiles where neither mode is fully right:')
for r in asm:
    m = r['mode_summary']; n = m['occurrences']
    if m['false_ok'] < n and m['true_ok'] < n:
        print(f"  {r['file']}  occ={n} f={m['false_ok']} t={m['true_ok']} {r['orientation']} {r['identity']} {syskey(r)}")

print('\nroot structure:')
print('  n_root_pds:', Counter(min(r['n_root_pds'], 5) for r in ok))
print('  files with solids not reachable from products:', sum(1 for r in ok if r['solids_unreached_from_products']))
uu = Counter()
for r in ok:
    for k, v in r.get('unreached_used_by', {}).items(): uu[k] += 1
print('  unreached solids referenced by (files):', uu.most_common(10))
print('  files with no PRODUCT_DEFINITION:', sum(1 for r in ok if not r['counts'].get('PRODUCT_DEFINITION')))
print('  files with mapped items:', sum(1 for r in ok if r['counts'].get('MAPPED_ITEM')), ' reached from PD reps:', sum(1 for r in ok if r['pd_reps_with_mapped_items']))
print('  files with MDGPR:', sum(1 for r in ok if r['counts'].get('MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION')))
print('  sdr targets:', sum((Counter(r['sdr_targets']) for r in ok), Counter()))
print('  products with several PDs (files):', sum(1 for r in ok if r['products_with_several_pds']))
print('  occurrences without IDT (op types etc):', sum((Counter({k: v for k, v in r['verdict'].items() if k.startswith('op:') or k in ('correct_unknown', 'no_axis2placement')}) for r in ok), Counter()))
print('  orientation special:', sum((Counter({k: v for k, v in r['orientation'].items() if k not in ('rep1=child', 'rep1=parent')}) for r in ok), Counter()))
print('  schemas:', Counter((r.get('schema') or '?').split(' ')[0] for r in ok).most_common(8))
