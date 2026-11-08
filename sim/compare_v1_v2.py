"""Old pipeline vs new pipeline, on the same simulators. Produces the numbers behind the
regression slides.

    python -m sim.compare_v1_v2 out_dir

Writes out_dir/compare.json and PNG figures. Experiments:

  link      N moves over a faulty link (USB-wake garble + byte drops): v1 bridge + v4.4
            firmware model vs v2 bridge + v5 firmware model. Counts wrong-target
            executions, ghost moves, false failures, re-CALs, port reloads, silent
            wrong positions.
  repeat    the same MOVE repeated with an occasional EL snag: final physical error,
            and whether the host was told.
  home      where "MOVE 0 0" physically lands after calibration, from different boot poses.
  sched     cumulative probability covered vs tiles: greedy pixel (v1) vs FOV-integrated (v2).
  detect    false alarms on clean fields and detections vs flux: v1 detector vs v2 pipeline.
"""
from __future__ import annotations

import json
import math
import os
import random
import socket
import sys
import tempfile
import time

import numpy as np

from airis import config
from airis.airis_bridge import Bridge
from airis.detect import Pipeline, render_reference
from airis.pointing import slew_time_s
from airis.scheduler import TileQueue
from airis.transport import SocketTransport
from sim.fake_arduino import FakeArduino, Faults
from sim.legacy import FakeArduinoV4, LegacyBridge, greedy_pixel_order, v1_transient_found
from sim.mount_sim import MountSim
from sim.sky_sim import FakeCamera, SkyModel

import healpy as hp

FAST = dict(ACK_TIMEOUT_S=0.3, MOVE_START_TIMEOUT_S=1.0, MOVE_MARGIN_S=1.0, MOVE_MAX_S=3.0,
            CAL_START_TIMEOUT_S=1.0, CAL_FINISH_TIMEOUT_S=5.0, LINK_SILENCE_S=1.5, KEEPALIVE_PERIOD_S=0.6)
TOL = config.SETTLE_TOL_DEG
HOME_PHYS = (0.0, 0.0)


def pair(cls, mount=None, **kw):
    a, b = socket.socketpair()
    mount = mount or MountSim.at_boot()
    fa = cls(b, mount, **kw)
    fa.start()
    return a, fa


def targets(n, seed):
    rng = random.Random(seed)
    return [(round(rng.uniform(-20, 80), 3), round(rng.uniform(-150, 150), 3)) for _ in range(n)]


def phys_err(mount, el, az):
    """Physical error of the mount w.r.t. the physical target of an encoder-frame command, given SETZERO ran."""
    pe, pa = mount.physical()
    return pe - el, (pa - az + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------- link experiment

def run_link_v1(n, faults, paced=False):
    sock, fa = pair(FakeArduinoV4, faults=faults, cal_duration_s=2.0)
    br = LegacyBridge(SocketTransport(sock), scale=25.0)
    br.run_cal()
    # v1 never anchored the frame: 0,0 is the index. Compare in the encoder frame instead
    # (what the operator typed), so link faults are isolated from the frame bug.
    out = dict(cmds=0, ok_reported=0, false_fail=0, wrong_target=0, ghost=0, recal=0, reload=0,
               silent_wrong=0, timeout=0, t=0.0, codes={})
    t0 = time.monotonic()
    for i, (el, az) in enumerate(targets(n, 11)):
        if i % 4 == 0:
            time.sleep(0.3)                        # 7.5 s of sim idle: the USB link has gone to sleep
        if paced:                                  # an operator waits for the mount to stop first
            while fa.mount.moving() or fa.state in ("MOVING", "HOMING"):
                time.sleep(0.02)
        before = len(fa.moves_executed)
        r = br.move(el, az)
        time.sleep(0.05)
        execd = fa.moves_executed[before:]
        out["cmds"] += 1
        wrong = [m for m in execd if abs(m[0] - el) > TOL or abs(m[1] - az) > TOL]
        out["wrong_target"] += len(wrong)
        out["ghost"] += max(0, len(execd) - 1)
        if r == "ok":
            out["ok_reported"] += 1
        elif r == "timeout":
            out["timeout"] += 1
        else:
            key = " ".join(r.split()[:3])
            out["codes"][key] = out["codes"].get(key, 0) + 1
            out["false_fail"] += int(any(abs(m[0] - el) <= TOL and abs(m[1] - az) <= TOL for m in execd))
        # where is the mount, really?
        while fa.mount.moving() or fa.state == "MOVING":
            time.sleep(0.02)
        e1 = fa.mount.el.pos - fa.mount.el.enc_offset - el
        e2 = fa.mount.az.pos - fa.mount.az.enc_offset - az
        if (abs(e1) > TOL or abs(e2) > TOL) and r == "ok":
            out["silent_wrong"] += 1
    out["recal"] = br.cals_sent - 1
    out["reload"] = br.reloads
    out["t"] = time.monotonic() - t0
    br.close(); fa.shutdown()
    return out


def run_link_v2(n, faults):
    for k, v in FAST.items():
        setattr(config, k, v)
    sock, fa = pair(FakeArduino, faults=faults, cal_duration_s=2.0)
    tmp = tempfile.mkdtemp()
    br = Bridge(SocketTransport(sock), moves_csv=os.path.join(tmp, "moves.csv"))
    assert br.calibrate().ok
    out = dict(cmds=0, ok_reported=0, false_fail=0, wrong_target=0, ghost=0, recal=0, reload=0,
               silent_wrong=0, timeout=0, naks=0, retries=0, t=0.0, codes={})
    t0 = time.monotonic()
    for i, (el, az) in enumerate(targets(n, 11)):
        if i % 4 == 0:
            time.sleep(0.3)
        before = len(fa.moves_executed)
        r = br.move(el, az)
        time.sleep(0.02)
        execd = fa.moves_executed[before:]
        out["cmds"] += 1
        out["wrong_target"] += sum(1 for m in execd if abs(m[0] - el) > TOL or abs(m[1] - az) > TOL)
        out["ghost"] += max(0, len(execd) - 1)
        out["retries"] += r.attempts - 1
        out["recal"] += int(r.recalibrated)
        if r.ok:
            out["ok_reported"] += 1
            if abs(r.el_err) > TOL or abs(r.az_err) > TOL:
                out["silent_wrong"] += 1
        elif r.code == "MNT-68":
            out["timeout"] += 1
        else:
            out["codes"][r.code] = out["codes"].get(r.code, 0) + 1
            out["false_fail"] += int(any(abs(m[0] - el) <= TOL and abs(m[1] - az) <= TOL for m in execd))
    out["naks"] = fa.naks
    out["t"] = time.monotonic() - t0
    br.close(); fa.shutdown()
    return out


def run_burst(kind, n):
    """n commands issued back to back, as the scheduler would, on a clean link: how long until all done,
    how many busy answers, how many CALs."""
    tgts = [(20.0 + 5 * i, -60.0 + 10 * i) for i in range(n)]
    if kind == "v1":
        sock, fa = pair(FakeArduinoV4, cal_duration_s=2.0)
        br = LegacyBridge(SocketTransport(sock), scale=25.0)
        br.run_cal()
        t0 = fa.mount.t
        results = [br.move(el, az) for el, az in tgts]
        while fa.mount.moving() or fa.state != "IDLE":
            time.sleep(0.02)
        out = dict(sim_s=fa.mount.t - t0, ok=sum(r == "ok" for r in results), busy=sum("busy" in l for l in fa.log),
                   cals=br.cals_sent - 1, executed=len(fa.moves_executed), reached_last=abs(fa.mount.az.enc_deg() - tgts[-1][1]) < TOL)
        br.close()
    else:
        for k, v in FAST.items():
            setattr(config, k, v)
        sock, fa = pair(FakeArduino, cal_duration_s=2.0)
        br = Bridge(SocketTransport(sock), moves_csv=os.path.join(tempfile.mkdtemp(), "moves.csv"))
        assert br.calibrate().ok
        n0 = len(fa.moves_executed)                 # the anchor move of calibrate() is not one of ours
        t0 = fa.mount.t
        results = [br.move(el, az) for el, az in tgts]
        out = dict(sim_s=fa.mount.t - t0, ok=sum(r.ok for r in results), busy=sum("busy" in l for l in fa.log),
                   cals=sum(r.recalibrated for r in results), executed=len(fa.moves_executed) - n0, reached_last=abs(fa.mount.az.enc_deg() - tgts[-1][1]) < TOL)
        br.close()
    fa.shutdown()
    return out


# ---------------------------------------------------------------- repeat experiment

def run_repeat(cls, bridge_kind, n, snag_prob, seed):
    rng = random.Random(seed)
    el, az = 40.0, 60.0
    mount = MountSim.at_boot()
    sock, fa = pair(cls, mount=mount, cal_duration_s=2.0)
    pts = []
    if bridge_kind == "v1":
        br = LegacyBridge(SocketTransport(sock), scale=25.0)
        br.run_cal()
        for i in range(n):
            br.move(0.0, 0.0); time.sleep(0.05)          # back to a start the snag is not in the way of
            while fa.mount.moving() or fa.state == "MOVING":
                time.sleep(0.02)
            mount.el.stall_at = (el + mount.el.enc_offset - 6.0) if rng.random() < snag_prob else None
            r = br.move(el, az)
            while fa.mount.moving() or fa.state == "MOVING":
                time.sleep(0.02)
            e1 = mount.el.pos - mount.el.enc_offset - el
            e2 = mount.az.pos - mount.az.enc_offset - az
            pts.append(dict(el_err=e1, az_err=e2, reported=(r != "ok"), result=r))
            mount.el.stall_at = None
        br.close()
    else:
        for k, v in FAST.items():
            setattr(config, k, v)
        br = Bridge(SocketTransport(sock), moves_csv=os.path.join(tempfile.mkdtemp(), "moves.csv"))
        assert br.calibrate().ok
        for i in range(n):
            br.move(0.0, 0.0); time.sleep(0.02)
            mount.el.stall_at = (el + mount.el.enc_offset - 6.0) if rng.random() < snag_prob else None
            r = br.move(el, az)
            e1 = mount.el.pos - mount.el.enc_offset - el
            e2 = mount.az.pos - mount.az.enc_offset - az
            pts.append(dict(el_err=e1, az_err=e2, reported=(not r.ok), result=r.code,
                            residual=(r.el_err, r.az_err)))
            mount.el.stall_at = None
        br.close()
    fa.shutdown()
    return pts


# ---------------------------------------------------------------- home experiment

def run_home(kind, sessions, seed):
    rng = random.Random(seed)
    pts = []
    for s in range(sessions):
        mount = MountSim.at_boot(el=rng.uniform(-30, 70), az=rng.uniform(-170, 170))
        mount.index_jitter_deg = 0.3
        if kind == "v1":
            sock, fa = pair(FakeArduinoV4, mount=mount, cal_duration_s=2.0)
            br = LegacyBridge(SocketTransport(sock), scale=25.0)
            br.run_cal()
            if s % 2:                      # a DTR reset from a port reopen, half the sessions
                fa.reset(); time.sleep(0.05)
            br.move(0.0, 0.0)
            while fa.mount.moving() or fa.state == "MOVING":
                time.sleep(0.02)
            br.close()
        else:
            for k, v in FAST.items():
                setattr(config, k, v)
            sock, fa = pair(FakeArduino, mount=mount, cal_duration_s=2.0)
            br = Bridge(SocketTransport(sock), moves_csv=os.path.join(tempfile.mkdtemp(), "moves.csv"))
            assert br.calibrate().ok
            if s % 2:
                fa.reset(); time.sleep(0.05)
            br.move(0.0, 0.0)
            br.close()
        pe, pa = mount.physical()
        pts.append(dict(el=pe, az=pa))
        fa.shutdown()
    return pts


# ---------------------------------------------------------------- scheduler experiment

def gaussian_map(nside, centres, sigmas, weights):
    npix = hp.nside2npix(nside)
    vec = np.array(hp.pix2vec(nside, np.arange(npix))).T
    p = np.zeros(npix)
    for (ra0, dec0), sg, w in zip(centres, sigmas, weights):
        v0 = hp.ang2vec(np.radians(90 - dec0), np.radians(ra0))
        d = np.degrees(np.arccos(np.clip(vec @ v0, -1, 1)))
        p += w * np.exp(-0.5 * (d / sg) ** 2)
    return p / p.sum()


def run_sched(n_tiles=12):
    nside = 64
    maps = {
        "broad (sigma 6 deg)": gaussian_map(nside, [(80, -5)], [6.0], [1.0]),
        "two lobes (3 + 4 deg)": gaussian_map(nside, [(80, -5), (96, 2)], [3.0, 4.0], [0.6, 0.4]),
        "sharp core + wide skirt": gaussian_map(nside, [(80, -5), (80, -5)], [1.0, 12.0], [0.3, 0.7]),
    }
    out = {}
    for name, prob in maps.items():
        q = TileQueue(n_candidates=3000)
        q.set_map(prob, 1, 0, nested=False)
        covered = np.zeros(prob.size, bool)
        # v2: greedy on FOV-integrated live probability (what rescore does, minus horizon/sun)
        cum2 = []
        for _ in range(n_tiles):
            live = np.where(covered, 0.0, prob)
            best, bp = None, 0.0
            for ra, dec, pix in q._candidate_centres()[:400]:
                m = q._footprint(ra, dec)
                v = float(live[m].sum())
                if v > bp:
                    best, bp = (ra, dec), v
            covered |= q._footprint(*best)
            q.covered = covered
            cum2.append(float(prob[covered].sum()))
        # v1: most probable pixel, then the next most probable pixel, footprint credited the same way
        covered1 = np.zeros(prob.size, bool)
        cum1 = []
        order = np.argsort(prob)[::-1]
        theta, phi = hp.pix2ang(nside, order)
        k = 0
        for _ in range(n_tiles):
            ra, dec = float(np.degrees(phi[k])), float(90 - np.degrees(theta[k]))
            k += 1
            covered1 |= q._footprint(ra, dec)
            cum1.append(float(prob[covered1].sum()))
        out[name] = dict(v1=cum1, v2=cum2)
    return out


# ---------------------------------------------------------------- detection experiment

def _v1_diff(cam, frame, stars, ra, dec, registered):
    """image_subtraction.py: reference on a flat grid (or, for the matrix, on the solved grid), plain subtraction."""
    wcs = cam.wcs_for(ra, dec)
    if not registered:
        wcs.rot_deg, wcs.flip_x = 0.0, False
    ref = render_reference(stars, wcs, frame.data.shape, frame.psf_sigma_px)
    scale = np.percentile(frame.data, 99.5) / max(np.percentile(ref, 99.5), 1e-9)
    return frame.data.astype(float) - ref * scale


def run_detect():
    sky = SkyModel(seed=2)
    pipe = Pipeline(csv_path=None)
    out = {}
    for psf in (1.2, 3.0):
        n_clean = 20
        fa = dict(v1=0, v1_registered=0, v2_unregistered=0, v2=0)
        for s in range(n_clean):
            cam = FakeCamera(sky, transient=None, rot_deg=2.0 * s - 10, flip_x=bool(s % 2), seed=300 + s, psf_sigma=psf)
            ra, dec = sky.ra0 + 0.5 * s, sky.dec0 + 0.3 * s
            a = cam.capture(ra, dec, 4.0); b = cam.capture(ra, dec, 4.0)
            stars = sky.catalog(ra, dec)
            fa["v1"] += int(v1_transient_found(_v1_diff(cam, a, stars, ra, dec, False)))
            fa["v1_registered"] += int(v1_transient_found(_v1_diff(cam, a, stars, ra, dec, True)))
            a_wrong = cam.capture(ra, dec, 4.0); a_wrong.wcs = cam.wcs_for(ra, dec); a_wrong.wcs.rot_deg, a_wrong.wcs.flip_x = 0.0, False
            fa["v2_unregistered"] += int(pipe.run(a_wrong, stars, second=b).transient_found)
            fa["v2"] += int(pipe.run(a, stars, second=b).transient_found)
        curve = {}
        for flux in (150.0, 300.0, 600.0, 1200.0, 2500.0, 5000.0):
            d = dict(v1=0, v1_registered=0, v2=0)
            n = 6
            for s in range(n):
                cam = FakeCamera(sky, transient=(sky.ra0 + 0.3, sky.dec0 + 0.1, flux), rot_deg=5.0, seed=400 + s, psf_sigma=psf)
                a = cam.capture(sky.ra0, sky.dec0, 4.0); b = cam.capture(sky.ra0, sky.dec0, 4.0)
                stars = sky.catalog(sky.ra0, sky.dec0)
                d["v1"] += int(v1_transient_found(_v1_diff(cam, a, stars, sky.ra0, sky.dec0, False)))
                d["v1_registered"] += int(v1_transient_found(_v1_diff(cam, a, stars, sky.ra0, sky.dec0, True)))
                res = pipe.run(a, stars, second=b)
                d["v2"] += int(res.transient_found and abs(res.candidates[0].ra - (sky.ra0 + 0.3)) < 0.05
                               and abs(res.candidates[0].dec - (sky.dec0 + 0.1)) < 0.05)
            curve[flux] = {k: v / n for k, v in d.items()}
        out[f"psf_{psf}"] = dict(clean_fields=n_clean, false_alarms=fa, curve=curve)
    return out


# ---------------------------------------------------------------- figures

def figures(res, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    V1, V2 = "#B85042", "#1C7293"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "axes.spines.top": False, "axes.spines.right": False})

    # repeat scatter: what did the host learn?
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharex=True, sharey=True)
    for ax, key, col, title in ((axes[0], "v1", V1, "v1 bridge + v4.4 firmware"), (axes[1], "v2", V2, "v2 bridge + v5 firmware")):
        pts = res["repeat"][key]
        off = [p for p in pts if abs(p["el_err"]) > TOL or abs(p["az_err"]) > TOL]
        good = [p for p in pts if abs(p["el_err"]) <= TOL and abs(p["az_err"]) <= TOL]
        ax.scatter([p["az_err"] for p in good], [p["el_err"] for p in good], s=40, color=col, label=f"on target: {len(good)}")
        ax.scatter([p["az_err"] for p in off], [p["el_err"] for p in off], s=110, marker="x", color="black", linewidths=2.5, label=f"snagged: {len(off)}")
        ax.add_patch(plt.Rectangle((-TOL, -TOL), 2 * TOL, 2 * TOL, fill=False, ls="--", color="gray"))
        ax.axhline(0, color="#ccc", lw=0.8); ax.axvline(0, color="#ccc", lw=0.8)
        if off:
            p = off[0]
            txt = ("host saw: 'ERR 50' / timeout\n(same code as busy, not homed, reset)" if key == "v1"
                   else f"host saw: MNT-58 SETTLE_FAIL\nresidual el={p['residual'][0]:+.1f} az={p['residual'][1]:+.1f}, logged to moves.csv")
            ax.annotate(txt, (p["az_err"], p["el_err"]), xytext=(1.5, -2.2), fontsize=10,
                        arrowprops=dict(arrowstyle="->", color="black"))
        ax.set_title(title); ax.set_xlabel("AZ error (deg, physical)"); ax.legend(loc="upper right", fontsize=10, frameon=False)
    axes[0].set_ylabel("EL error (deg, physical)")
    axes[0].set_xlim(-4, 8); axes[0].set_ylim(-8, 2.5)
    fig.suptitle(f"Same MOVE 40 60, {len(res['repeat']['v1'])} repeats, EL snag 6 deg short on {int(res['repeat']['snag_prob']*100)} % of moves", y=1.02)
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig_repeat.png"), dpi=170, bbox_inches="tight"); plt.close(fig)

    # home scatter
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    for key, col, lab in (("v1", V1, "v1: CAL only"), ("v2", V2, "v2: CAL + MOVE -55 -35 + SETZERO")):
        pts = res["home"][key]
        ax.scatter([p["az"] for p in pts], [p["el"] for p in pts], s=70, color=col, label=lab, alpha=0.85)
    ax.scatter([0], [0], marker="+", s=300, color="black", linewidths=2, label="intended physical home")
    ax.scatter([35], [55], marker="*", s=200, color="gray", label="encoder index pulse")
    ax.set_xlabel("physical AZ where 'MOVE 0 0' stopped (deg)"); ax.set_ylabel("physical EL (deg)")
    ax.set_xlim(-20, 60); ax.set_ylim(-15, 75); ax.legend(frameon=False, fontsize=10, loc="upper left")
    ax.set_title(f"Where home is, {len(res['home']['v1'])} sessions from random boot poses (half with a mid-run reset)")
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig_home.png"), dpi=170, bbox_inches="tight"); plt.close(fig)


def main(out):
    os.makedirs(out, exist_ok=True)
    res = {}
    faults = Faults(garble_after_idle_s=2.0, garble_bytes=2, drop_byte_prob=0.01, seed=5)
    print("link v1 ..."); res["link_v1"] = run_link_v1(60, faults); print(res["link_v1"])
    print("burst ..."); res["burst"] = dict(v1=run_burst("v1", 12), v2=run_burst("v2", 12)); print(res["burst"])
    print("link v2 ..."); res["link_v2"] = run_link_v2(60, Faults(garble_after_idle_s=2.0, garble_bytes=2, drop_byte_prob=0.01, seed=5)); print(res["link_v2"])
    print("repeat ..."); res["repeat"] = dict(v1=run_repeat(FakeArduinoV4, "v1", 30, 0.2, 21), v2=run_repeat(FakeArduino, "v2", 30, 0.2, 21), snag_prob=0.2)
    print("home ..."); res["home"] = dict(v1=run_home("v1", 12, 31), v2=run_home("v2", 12, 31))
    print("sched ..."); res["sched"] = run_sched()
    print("detect ..."); res["detect"] = run_detect(); print(res["detect"])
    json.dump(res, open(os.path.join(out, "compare.json"), "w"), indent=1, default=float)
    figures(res, out)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "compare_out")
