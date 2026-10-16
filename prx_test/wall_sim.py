#!/usr/bin/env python3
"""
Virtual-wall proximity test driver for ArduCopter SITL (Loiter + simple avoidance).

 * launches arducopter SITL with the indoor (vision/vicon) configuration
 * reads the true vehicle state from SIMSTATE and synthesises a 72-ray 2-D lidar scan
   against a virtual wall (plane N = --wall-n), sent as MAVLink OBSTACLE_DISTANCE
   at --rate Hz with configurable latency / noise / FOV / drop-out / min range
 * drives the pilot stick (RC override, pitch channel) through the test phases
       T1  : approach the wall from far away with the stick held forward
       REL : release the stick, hover in front of the wall
       T2  : push the stick forward again from the hover
 * logs truth + what the sensor sent + what AP reports (DISTANCE_SENSOR) to CSV and
   prints/stores per-phase metrics (min distance, hold distance, oscillation, back-out time)
"""
import argparse, csv, json, math, os, random, subprocess, sys, time
from collections import deque
from pymavlink import mavutil

M_PER_DEG = 111319.5

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

class WallSensor:
    """72-ray scan against an infinite wall plane N = wall_n (vehicle treated as a point)."""
    def __init__(self, a):
        self.a = a
        self.n_rays = 72
        self.inc = 360.0 / self.n_rays
        self.queue = deque()  # (send_at_wallclock, msg_fields)
        self.rng = random.Random(a.seed)

    def scan(self, n, e, yaw_deg):
        a = self.a
        d_perp = a.wall_n - n
        dists = []
        for i in range(self.n_rays):
            body_ang = i * self.inc  # clockwise from nose
            if abs(wrap180(body_ang)) > a.fov / 2.0:
                dists.append(65535); continue
            bearing = math.radians(yaw_deg + body_ang)
            c = math.cos(bearing)
            if c <= 1e-6 or d_perp <= 0.0:
                dists.append(65535); continue
            d = d_perp / c
            if a.noise > 0:
                d += self.rng.gauss(0.0, a.noise)
            if self.rng.random() < a.dropout:
                dists.append(65535); continue
            if d > a.max_dist:
                dists.append(65535); continue
            if d < a.min_dist:
                if a.under_min == 'clamp':
                    d = a.min_dist
                elif a.under_min == 'drop':
                    dists.append(65535); continue
                # 'raw': send the value as-is; AP discards values < min_distance
            dists.append(max(0, min(65534, int(round(d * 100.0)))))
        return dists

    def push(self, now, time_boot_ms, n, e, yaw_deg):
        dists = self.scan(n, e, yaw_deg)
        self.queue.append((now + self.a.delay, time_boot_ms, dists))
        return dists

    def pop_due(self, now):
        out = []
        while self.queue and self.queue[0][0] <= now:
            out.append(self.queue.popleft())
        return out


class Runner:
    def __init__(self, a):
        self.a = a
        self.sitl = None
        self.master = None
        self.t0 = None
        self.phase = 'INIT'
        self.pitch_pwm = 1500
        self.thr_pwm = 1500
        self.rows = []
        self.last_ap_front = float('nan')   # DISTANCE_SENSOR orientation 0 current_distance (m)
        self.last_sent_front = float('nan')
        self.ekf_n = float('nan'); self.ekf_vx = float('nan')
        self.lat0 = a.home[0]; self.lon0 = a.home[1]
        self.alt_rel = 0.0
        self.sensor = WallSensor(a)
        self.last_sim = None  # (time_boot_ms, n, e, yaw)
        self.statustext = []

    # ---------------- SITL ----------------
    def launch(self):
        a = self.a
        os.makedirs(a.outdir, exist_ok=True)
        home = "%.7f,%.7f,%.1f,%.1f" % tuple(a.home)
        a.binary = os.path.abspath(a.binary)
        a.defaults = ",".join(os.path.abspath(x) for x in a.defaults.split(","))
        a.outdir = os.path.abspath(a.outdir)
        extra = os.path.join(a.outdir, "extra.parm")
        with open(extra, "w") as f:
            for kv in a.param:
                f.write("%s %s\n" % tuple(kv.split("=", 1)))
        a.defaults += "," + extra
        cmd = [a.binary, "--model", "quad", "-I%d" % a.instance, "-w", "--speedup", str(a.speedup),
               "--home", home, a.vicon_arg,
               "--defaults", a.defaults]
        self.sitl_log = open(os.path.join(a.outdir, "sitl.log"), "w")
        self.sitl = subprocess.Popen(cmd, cwd=a.outdir, stdout=self.sitl_log, stderr=subprocess.STDOUT)
        time.sleep(2.0)
        self.master = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % (5760 + 10 * a.instance), source_system=255)
        self.master.wait_heartbeat(timeout=60)
        print("heartbeat from sys %d" % self.master.target_system)

    def stop(self):
        if self.sitl:
            self.sitl.terminate()
            try: self.sitl.wait(5)
            except Exception: self.sitl.kill()

    # ---------------- helpers ----------------
    def send_rc(self):
        m = self.master.mav
        m.rc_channels_override_send(self.master.target_system, self.master.target_component,
                                    1500, self.pitch_pwm, self.thr_pwm, 1500, 1700, 1500, 1500, 1500)

    def set_interval(self, msg_id, hz):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                          msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def handle(self, msg):
        t = msg.get_type()
        if t == 'SIMSTATE':
            n = (msg.lat - self.lat0 * 1e7) * 1e-7 * M_PER_DEG
            e = (msg.lng - self.lon0 * 1e7) * 1e-7 * M_PER_DEG * math.cos(math.radians(self.lat0))
            self.last_sim = (time.time(), n, e, math.degrees(msg.yaw))
        elif t == 'LOCAL_POSITION_NED':
            self.ekf_n = msg.x; self.ekf_vx = msg.vx; self.alt_rel = -msg.z
        elif t == 'DISTANCE_SENSOR':
            if msg.orientation == 0:
                self.last_ap_front = msg.current_distance / 100.0
        elif t == 'STATUSTEXT':
            self.statustext.append((time.time() - (self.t0 or time.time()), msg.text))
            print("  STATUSTEXT: %s" % msg.text)
        elif t == 'HEARTBEAT':
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode = mavutil.mode_string_v10(msg)

    def pump(self, dt):
        """process incoming messages for dt seconds, sending sensor + rc at their rates"""
        end = time.time() + dt
        while time.time() < end:
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self.handle(msg)
            now = time.time()
            if now - self.last_rc_t >= 0.1:
                self.send_rc(); self.last_rc_t = now
            if self.sensor_on and self.last_sim and now - self.last_scan_t >= 1.0 / self.a.rate:
                self.last_scan_t = now
                _, n, e, yaw = self.last_sim
                self.sensor.push(now, int((now - self.t0) * 1000), n, e, yaw)
            for (_, tb, dists) in self.sensor.pop_due(now):
                self.master.mav.obstacle_distance_send(
                    int(tb) * 1000, mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, dists,
                    int(self.sensor.inc), int(self.a.min_dist * 100), int(self.a.max_dist * 100),
                    0.0, 0.0, mavutil.mavlink.MAV_FRAME_BODY_FRD)
                self.last_sent_front = dists[0] / 100.0 if dists[0] != 65535 else float('nan')
            if self.last_sim and now - self.last_log_t >= 0.02 and self.t0:
                self.last_log_t = now
                _, n, e, yaw = self.last_sim
                self.rows.append(dict(t=round(now - self.t0, 3), phase=self.phase, n=round(n, 3), e=round(e, 3),
                                      yaw=round(yaw, 1), d_true=round(self.a.wall_n - n, 3),
                                      d_sent_front=self.last_sent_front, d_ap_front=self.last_ap_front,
                                      ekf_n=round(self.ekf_n, 3), ekf_vx=round(self.ekf_vx, 3),
                                      alt=round(self.alt_rel, 2), pitch_pwm=self.pitch_pwm, mode=getattr(self, 'mode', '')))

    def wait_text(self, substr, timeout):
        n0 = len(self.statustext); end = time.time() + timeout
        while time.time() < end:
            self.pump(0.2)
            if any(substr in s for _, s in self.statustext[n0:]):
                return True
        return False

    # ---------------- flight ----------------
    def setup(self):
        a = self.a
        self.armed = False; self.mode = ''
        self.last_rc_t = 0; self.last_scan_t = 0; self.last_log_t = 0; self.sensor_on = False
        self.t0 = time.time()
        self.master.mav.system_time_send(int(time.time() * 1e6), 0)
        self.master.mav.set_gps_global_origin_send(self.master.target_system, int(a.home[0] * 1e7), int(a.home[1] * 1e7), int(a.home[2] * 1000))
        for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE, 50), (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                        (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)):
            self.set_interval(mid, hz)
        self.thr_pwm = 1000
        self.master.set_mode('LOITER')
        print("waiting for EKF/vision position and ready-to-arm ...")
        self.pump(1.0)
        # wait for EKF position
        end = time.time() + 120
        while time.time() < end and (math.isnan(self.ekf_n) or self.last_sim is None):
            self.pump(0.5)
        self.master.mav.set_gps_global_origin_send(self.master.target_system, int(a.home[0] * 1e7), int(a.home[1] * 1e7), int(a.home[2] * 1000))
        # settle, then arm (retry)
        self.pump(8.0)
        self.sensor_on = True
        for i in range(40):
            self.master.set_mode('LOITER')
            self.master.arducopter_arm()
            self.pump(2.0)
            if self.armed:
                break
        if not self.armed:
            raise RuntimeError("could not arm")
        print("armed, mode=%s  t=%.1f" % (self.mode, time.time() - self.t0))
        # take off in Loiter on throttle
        self.thr_pwm = 1700
        while self.alt_rel < a.alt:
            self.pump(0.2)
        self.thr_pwm = 1500
        self.pump(4.0)
        print("airborne alt=%.2f  d_wall=%.2f" % (self.alt_rel, a.wall_n - self.last_sim[1]))

    def run_phases(self):
        a = self.a
        plan = [('T1', a.stick_pwm, a.t1), ('REL', 1500, a.rel), ('T2', a.stick_pwm, a.t2), ('END', 1500, 4)]
        for name, pwm, dur in plan:
            self.phase = name; self.pitch_pwm = pwm
            print("phase %-3s pitch=%d for %gs  (d=%.2f)" % (name, pwm, dur, a.wall_n - self.last_sim[1]))
            self.pump(dur)
        self.phase = 'DONE'

    # ---------------- analysis ----------------
    def metrics(self):
        a = self.a
        res = {}
        for ph in ('T1', 'REL', 'T2'):
            rows = [r for r in self.rows if r['phase'] == ph]
            if not rows:
                continue
            d = [r['d_true'] for r in rows]; t = [r['t'] for r in rows]
            tend = t[-1]
            hold = [r['d_true'] for r in rows if r['t'] >= tend - a.hold_window]
            m = dict(min_d=min(d), t_min=t[d.index(min(d))] - t[0],
                     hold_mean=sum(hold) / len(hold), hold_min=min(hold), hold_max=max(hold),
                     osc_pp=max(hold) - min(hold))
            # breach / back-out
            breach_i = next((i for i, x in enumerate(d) if x < a.margin), None)
            if breach_i is None:
                m.update(breach=False, backout_s=None, below_margin_s=0.0)
            else:
                rec_i = next((i for i in range(breach_i, len(d)) if d[i] >= a.margin), None)
                m.update(breach=True, t_breach=t[breach_i] - t[0],
                         backout_s=(t[rec_i] - t[breach_i]) if rec_i is not None else None,
                         below_margin_s=sum(1 for x in d if x < a.margin) * 0.02)
            res[ph] = m
        return res

    def save(self):
        a = self.a
        with open(os.path.join(a.outdir, "truth.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader(); w.writerows(self.rows)
        m = self.metrics()
        with open(os.path.join(a.outdir, "metrics.json"), "w") as f:
            json.dump(dict(args=vars(a), metrics=m, statustext=self.statustext), f, indent=1)
        print(json.dumps(m, indent=1))
        return m


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', default='build/sitl/bin/arducopter')
    p.add_argument('--defaults', default='Tools/autotest/default_params/copter.parm,prx_test/indoor.parm')
    p.add_argument('--outdir', required=True)
    p.add_argument('--home', type=float, nargs=4, default=[-35.363261, 149.165230, 584.0, 0.0])
    p.add_argument('--speedup', type=float, default=1.0)
    p.add_argument('--instance', type=int, default=0)
    p.add_argument('--vicon-arg', default='--serial5=sim:vicon:', help='4.2: --uartF=sim:vicon:')
    p.add_argument('--param', action='append', default=[], help='extra NAME=VALUE applied at boot (e.g. PRX_FILT=0)')
    p.add_argument('--alt', type=float, default=1.5)
    p.add_argument('--wall-n', type=float, default=7.0, help='wall plane at N = this many metres from home')
    p.add_argument('--margin', type=float, default=0.5, help='AVOID_MARGIN used for metric evaluation')
    # sensor model
    p.add_argument('--rate', type=float, default=15.0, help='OBSTACLE_DISTANCE rate Hz')
    p.add_argument('--delay', type=float, default=0.0, help='sensor latency s')
    p.add_argument('--noise', type=float, default=0.0, help='gaussian range noise std m')
    p.add_argument('--fov', type=float, default=360.0, help='total horizontal FOV deg')
    p.add_argument('--dropout', type=float, default=0.0, help='probability a ray is missing')
    p.add_argument('--min-dist', type=float, default=0.2)
    p.add_argument('--max-dist', type=float, default=12.0)
    p.add_argument('--under-min', choices=['raw', 'clamp', 'drop'], default='raw')
    p.add_argument('--seed', type=int, default=1)
    # stick plan
    p.add_argument('--stick-pwm', type=int, default=1167, help='pitch pwm (1167 ~ 2/3 stick ~ 20deg of ANGLE_MAX 30)')
    p.add_argument('--t1', type=float, default=30)
    p.add_argument('--rel', type=float, default=8)
    p.add_argument('--t2', type=float, default=20)
    p.add_argument('--hold-window', type=float, default=6.0)
    a = p.parse_args()

    r = Runner(a)
    try:
        r.launch()
        r.setup()
        r.run_phases()
        return r.save()
    finally:
        r.stop()

if __name__ == '__main__':
    main()
