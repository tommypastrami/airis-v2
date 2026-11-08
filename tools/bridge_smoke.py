"""Bench acceptance for a bridge (real or simulated): cal, move, busy, sun.

    python -m tools.bridge_smoke /tmp/airis_bridge.sock
"""
import sys
import threading

from airis.bridge_client import MountClient


def main(path: str) -> int:
    c = MountClient(path, timeout_s=300.0)
    r = c.calibrate()
    print("cal", "ok" if r.ok else f"FAILED {r.code} {r.msg}")
    if not r.ok:
        return 1
    out = {}
    t = threading.Thread(target=lambda: out.setdefault("a", c.move(45.0, 30.0)))
    t.start()
    import time; time.sleep(0.3)
    b = c.move(-45.0, -30.0)
    t.join()
    a = out["a"]
    print("move", "ok" if a.ok else f"FAILED {a.code}", f"{a.el_err:.3f} {a.az_err:.3f} {a.duration_s:.2f}")
    print("busy", b.code)
    s = c.set_sun(-10.0, 120.0)
    print("sun", "ok" if s.ok else f"FAILED {s.code}")
    return 0 if a.ok and b.code == "MNT-63" and s.ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/airis_bridge.sock"))
