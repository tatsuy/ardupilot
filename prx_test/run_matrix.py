#!/usr/bin/env python3
"""run a matrix of wall_sim.py configs, N in parallel on different SITL instances"""
import itertools, json, os, queue, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
cases = []
BINS = os.environ.get('BINS', 'base,c1,c2,c12').split(',')
EXTRA = [x for x in os.environ.get('EXTRA', '').split(',') if x]
WORKERS = int(os.environ.get('WORKERS', '3'))
MORE = os.environ.get('MORE', '').split()  # extra wall_sim.py args, e.g. --defaults ... --vicon-arg ...
for binname in BINS:
    for stick in (1167, 1333):
        for filt in sys.argv[2:] or ('0.25',):
            cases.append((binname, stick, filt))
outroot = sys.argv[1]
free = queue.Queue()
for i in [int(x) for x in os.environ.get('INSTANCES', '0,1,2').split(',')]: free.put(i)
def run(ix_case):
    ix, (binname, stick, filt) = ix_case
    name = "%s_s%d_f%s%s" % (binname, stick, filt, ''.join('_' + e.replace('=', '') for e in EXTRA))
    out = os.path.join(outroot, name)
    inst = free.get()
    cmd = [sys.executable, os.path.join(HERE, 'wall_sim.py'), '--binary', os.path.join(HERE, 'arducopter_' + binname),
           '--outdir', out, '--instance', str(inst), '--stick-pwm', str(stick), '--param', 'PRX_FILT=%s' % filt] + sum([['--param', e] for e in EXTRA], []) + [
           '--t1', '30', '--rel', '8', '--t2', '20'] + MORE
    with open(out + '.log', 'w') if os.makedirs(out, exist_ok=True) or True else None as f:
        try:
            rc = subprocess.call(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, timeout=400)
        except subprocess.TimeoutExpired:
            rc = -1
        finally:
            free.put(inst)
    try:
        m = json.load(open(os.path.join(out, 'metrics.json')))['metrics']
    except Exception as e:
        m = {'error': str(e)}
    print(name, 'rc=%d' % rc, json.dumps({k: (round(v['min_d'], 2), v.get('backout_s')) for k, v in m.items() if isinstance(v, dict)}), flush=True)
    return name, m
with ThreadPoolExecutor(max_workers=WORKERS) as ex:
    # keep instance ids unique among concurrently running jobs: ix%3 with 3 workers in submission order
    results = dict(ex.map(run, enumerate(cases)))
json.dump(results, open(os.path.join(outroot, 'all_metrics.json'), 'w'), indent=1)
