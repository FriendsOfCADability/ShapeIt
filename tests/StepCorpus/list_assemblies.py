"""Extract the assembly files from the results of stepstruct.py.

usage: list_assemblies.py <struct.jsonl> <filelist out> <orientation.tsv out>
Writes the files with assembly occurrences, sorted by size (small files first), as input for
runcorpus.py, and a file<TAB>orientation list (rep1=child / rep1=parent) for compare.py --orientation.
"""
import json, sys

rows = [json.loads(l) for l in open(sys.argv[1], encoding='utf-8')]
rows = [r for r in rows if 'error' not in r]
asm = [r for r in rows if r['mode_summary']['occurrences'] > 0]
asm.sort(key=lambda r: r['size'])
with open(sys.argv[2], 'w', encoding='utf-8') as f:
    for r in asm: f.write(r['file'] + '\n')
with open(sys.argv[3], 'w', encoding='utf-8') as f:
    for r in asm: f.write(r['file'] + '\t' + '+'.join(sorted(r['orientation'])) + '\n')
print(len(asm), 'files,', sum(r['size'] for r in asm) / 1e6, 'MB')
