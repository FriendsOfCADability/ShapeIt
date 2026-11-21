"""Run StepProbe over a list of STEP files.

usage: runcorpus.py <StepProbe.exe> <filelist> <outdir> <label> [workers] [timeout_s]
<filelist> has one STEP file path per line. Writes <outdir>/<n>.<label>.txt per file and
<outdir>/index.tsv (n, file). Existing outputs are kept, so an interrupted run can be resumed.
A probe that dies without output is recorded as "# CRASH", one that runs too long as "# TIMEOUT".
Several labels (comma separated) run every file once per label.
"""
import subprocess, sys, os
from concurrent.futures import ThreadPoolExecutor

exe, flist, outdir, labels = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4].split(',')
workers = int(sys.argv[5]) if len(sys.argv) > 5 else 3
timeout = int(sys.argv[6]) if len(sys.argv) > 6 else 900
files = [l.strip() for l in open(flist, encoding='utf-8-sig') if l.strip()]
os.makedirs(outdir, exist_ok=True)
with open(os.path.join(outdir, 'index.tsv'), 'w', encoding='utf-8') as f:
    for n, fn in enumerate(files): f.write(f'{n}\t{fn}\n')


def run(job):
    n, fn, label = job
    out = os.path.join(outdir, f'{n}.{label}.txt')
    if os.path.exists(out): return
    try:
        subprocess.run([exe, fn, label, out], timeout=timeout, capture_output=True)
        if not os.path.exists(out):
            open(out, 'w').write('# CRASH\n')
    except subprocess.TimeoutExpired:
        open(out, 'w').write('# TIMEOUT\n')


jobs = [(n, fn, l) for n, fn in enumerate(files) for l in labels]
with ThreadPoolExecutor(workers) as ex:
    list(ex.map(run, jobs))
print('done', len(jobs))
