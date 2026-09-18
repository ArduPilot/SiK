#!/usr/bin/env python3
"""Before/after link measurements for several firmware builds at one radio configuration.

Usage: benchmark.py IMG_DIR ECC AIR_SPEED [PARTS]

IMG_DIR holds one image per build in PLAN below, named after its branch. Each is
flashed onto both radios (SIK_TX / SIK_RX, default /dev/ttyUSB0 and /dev/ttyUSB1),
then the parts listed for it run:
  A  bursts of 4 split messages, and the one-way limit for split messages
  B  one-way and two-way limits for whole messages, and one-way delay
  C  split messages in bursts while the receiving side sends heartbeats, so its
     radio injects RADIO_STATUS
PARTS limits the run, e.g. "C".
Rates scale with the link capacity (x2 without ECC, x air/64) and are capped at the
57600 serial limit. Without ECC a radio packet holds 250 bytes, so the split and whole
message sizes are larger too.
"""
import os, subprocess, sys, time
from sikat import at
IMG, ECC, AIR = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
HERE = os.path.dirname(os.path.abspath(__file__))
UP = os.path.join(HERE, "..", "uploader.py")
TX = os.environ.get("SIK_TX", "/dev/ttyUSB0")
RX = os.environ.get("SIK_RX", "/dev/ttyUSB1")
F = (AIR / 64.0) * (1 if ECC else 2)
SPLIT, WHOLE = (106, 93) if ECC else (240, 200)
CAP = 5400
rate = lambda r: int(min(CAP, r * F))

def sh(args):
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    return (r.stdout + r.stderr).strip().splitlines()

def configure():
    for p in (RX, TX):
        at(p, [f"ATS5={ECC}", f"ATS2={AIR}", "AT&W", "ATZ"], quiet=True)
    time.sleep(6)
    for p in (TX, RX):
        o = at(p, ["ATS5?", "ATS2?"], quiet=True)
        print(f"  {p}: ECC={o['ATS5?'].split()[-1]} AIR_SPEED={o['ATS2?'].split()[-1]}", flush=True)

def flash(img):
    for p in (TX, RX):
        out = "\n".join(sh(["python3", UP, "--port", p, f"{IMG}/{img}.ihx"])).replace("\r", "\n")
        if "done" not in out:
            # measuring whatever is on the radio under this label would be misleading
            raise SystemExit(f"  FLASH FAILED on {p}, stopping")
    time.sleep(6)
    print(f"  ATI6: {' '.join(at(TX, ['ATI6'], quiet=True)['ATI6'].split()[1:])}", flush=True)

def lt(*args):
    return sh(["python3", os.path.join(HERE, "linktest.py"), "--tx", TX, "--rx", RX, *args])[-1]

def tests(which):
    if "A" in which:
        print("  [bursts] " + lt("--payload", SPLIT, "--rate", rate(800), "--burst", 4, "--secs", 20), flush=True)
        for r in sorted({rate(x) for x in (1000, 1200, 1600, 2000, 2400, 3200)}):
            print(f"  [1way {SPLIT+12}B] " + lt("--payload", SPLIT, "--rate", r, "--secs", 20), flush=True)
    if "B" in which:
        for r in sorted({rate(x) for x in (1000, 1200, 1600, 2000, 2400, 3200)}):
            print(f"  [1way {WHOLE+12}B] " + lt("--payload", WHOLE, "--rate", r, "--secs", 20), flush=True)
        for r in sorted({min(2700, rate(x)) for x in (1000, 1600, 2400)}):
            for l in sh(["python3", os.path.join(HERE, "duplex.py"), "--a", TX, "--b", RX, "--payload-a", WHOLE, "--payload-b", WHOLE, "--rate-a", r, "--rate-b", r])[-2:]:
                print(f"  [2way {WHOLE+12}B @{r}] " + l, flush=True)
        for load in (0, rate(1000)):
            print("  [latency] " + sh(["python3", os.path.join(HERE, "latency.py"), "--tx", TX, "--rx", RX, "--load", load])[-1], flush=True)
    if "C" in which:
        for rep in (1, 2):
            print(f"  [heartbeat {SPLIT+12}B bursts] " + lt("--payload", SPLIT, "--rate", rate(800), "--burst", 4, "--secs", 60, "--heartbeat"), flush=True)

print(f"#### ECC={ECC} AIR_SPEED={AIR} (rate factor {F})", flush=True)
configure()
# each build and the parts that compare it with the one before
PLAN = (("master", "A"),
        ("tdm-bonus-window", "AB"),
        ("tdm-max-xmit-wrap", "AB"),
        ("tdm-defer-hop", "ABC"),
        ("radio-status-frame-boundary", "C"))
if len(sys.argv) > 4:
    # only rerun some parts, e.g. "C" for the RADIO_STATUS test alone
    PLAN = tuple((img, w) for img, w in ((i, "".join(c for c in w if c in sys.argv[4])) for i, w in PLAN) if w)
for img, which in PLAN:
    print(f"== {img}", flush=True)
    flash(img)
    tests(which)
