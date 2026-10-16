#!/usr/bin/env python3
"""summarise metrics.json files from run_matrix.py into a markdown table (several runs of the same
config are shown as 'run1 / run2 ...')"""
import glob, json, os, sys
from collections import OrderedDict

NAMES = {'base': '変更前', 'c1': '変更1', 'c2': '変更2', 'c12': '変更1+2'}
STICK = {1167: '2/3 (≈20°)', 1333: '1/3 (≈10°)'}
ORDER = {'base': 0, 'c1': 1, 'c2': 2, 'c12': 3}

def fmt(v, nd=2):
    return '—' if v is None else ('%.' + str(nd) + 'f') % v

groups = OrderedDict()
for root in sys.argv[1:]:
    for f in sorted(glob.glob(os.path.join(root, '*/metrics.json'))):
        d = json.load(open(f))
        a = d['args']; m = d['metrics']
        binname = os.path.basename(a['binary']).replace('arducopter_', '')
        filt = [p.split('=')[1] for p in a['param'] if p.startswith('PRX_FILT=')]
        filt = float(filt[0]) if filt else 0.25
        groups.setdefault((filt, a['stick_pwm'], binname), []).append(m)

def col(ms, ph, fn):
    return ' / '.join(fn(m.get(ph, {}), m, ph) for m in ms)

PH = ['T1', 'REL', 'T2']
def crashed(m, ph='T1'):
    """True if the vehicle went through the wall in this phase or an earlier one"""
    return any(m.get(p, {}).get('min_d', 1) < 0 for p in PH[:PH.index(ph) + 1])

def mind(ph, m, name):
    return fmt(ph.get('min_d'))

def hold(ph, m, name):
    if not ph or crashed(m, name):
        return '—'
    return '%s (%s)' % (fmt(ph['hold_mean']), fmt(ph['osc_pp']))

def back(ph, m, name):
    if not ph or crashed(m, name):
        return '—'
    if not ph.get('breach'):
        return 'なし'
    return (fmt(ph['backout_s'], 1) + 's') if ph['backout_s'] is not None else '未回復'

def t2(fn):
    return lambda ph, m, name: '衝突(突き抜け)' if crashed(m, name) else fn(ph, m, name)

print('| PRX_FILT | スティック | バイナリ | T1 最小距離 [m] | T1 保持距離 (揺れ幅) [m] | T1 後退時間 | REL 保持距離 (揺れ幅) [m] | T2 最小距離 [m] | T2 保持距離 (揺れ幅) [m] | T2 後退時間 |')
print('|---|---|---|---|---|---|---|---|---|---|')
for (filt, stick, binname) in sorted(groups, key=lambda k: (k[0], -k[1], ORDER.get(k[2], 9))):
    ms = groups[(filt, stick, binname)]
    print('| %g | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
        filt, STICK.get(stick, stick), NAMES.get(binname, binname),
        col(ms, 'T1', mind), col(ms, 'T1', hold), col(ms, 'T1', back),
        col(ms, 'REL', hold),
        col(ms, 'T2', t2(mind)), col(ms, 'T2', t2(hold)), col(ms, 'T2', t2(back))))
