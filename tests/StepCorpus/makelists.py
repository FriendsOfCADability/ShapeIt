"""Create the file lists of the two corpus scenarios.

usage: makelists.py <corpus root> <outdir> [--max-mb 10]
Searches <corpus root> recursively for STEP files (*.stp, *.step, *.p21, any case) and writes
  <outdir>/quick.txt  the files smaller than --max-mb megabytes (default 10): the regular check of a change
  <outdir>/full.txt   all files: the complete check, which takes several hours
one path per line, sorted, so the numbering of runcorpus.py is the same for runs with the same list.
The lists contain the paths of the (customer) files, keep them out of the repository.
"""
import os, sys

args = sys.argv[1:]
max_mb = 10.0
if '--max-mb' in args:
    k = args.index('--max-mb')
    max_mb = float(args[k + 1])
    del args[k:k + 2]
if len(args) != 2:
    print(__doc__)
    sys.exit(1)
root, outdir = args
files = []
for d, _, names in os.walk(root):
    for name in names:
        if os.path.splitext(name)[1].lower() in ('.stp', '.step', '.p21'):
            path = os.path.join(d, name)
            files.append((path, os.path.getsize(path)))
files.sort()
os.makedirs(outdir, exist_ok=True)
quick = [p for p, s in files if s < max_mb * 1024 * 1024]
with open(os.path.join(outdir, 'quick.txt'), 'w', encoding='utf-8') as f:
    f.writelines(p + '\n' for p in quick)
with open(os.path.join(outdir, 'full.txt'), 'w', encoding='utf-8') as f:
    f.writelines(p + '\n' for p, s in files)
mb = lambda lst: sum(s for p, s in lst) / 2**20
print(f'quick: {len(quick)} files, {mb([x for x in files if x[1] < max_mb * 2**20]):.0f} MB')
print(f'full:  {len(files)} files, {mb(files):.0f} MB')
