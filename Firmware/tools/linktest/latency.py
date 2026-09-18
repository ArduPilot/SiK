#!/usr/bin/env python3
"""Measure one-way delay through the radio pair: send small timestamped frames, read them on the other port."""
import argparse, threading, time, serial, statistics
from linktest import build_frame, parse_stream

ap = argparse.ArgumentParser()
ap.add_argument("--tx", default="/dev/ttyUSB0"); ap.add_argument("--rx", default="/dev/ttyUSB1")
ap.add_argument("--hz", type=float, default=10); ap.add_argument("--secs", type=float, default=20)
ap.add_argument("--payload", type=int, default=20)
ap.add_argument("--load", type=int, default=0, help="B/s of 106-byte frames sent the same way as background")
a = ap.parse_args()

sent_t, recv_t, stop = {}, {}, threading.Event()
PING_BASE = 1 << 30   # counters for timed frames, kept apart from the load frames

def reader():
    with serial.Serial(a.rx, 57600, timeout=0.001) as r:
        r.reset_input_buffer(); buf = b""
        while not stop.is_set():
            d = r.read(4096)
            if d:
                now = time.monotonic()
                buf, got = parse_stream(buf + d, None)
                for c in got:
                    if c >= PING_BASE and c not in recv_t:
                        recv_t[c] = now
t = threading.Thread(target=reader, daemon=True); t.start()
time.sleep(0.5)
with serial.Serial(a.tx, 57600, timeout=0) as w:
    start = time.monotonic(); nping = nload = 0
    next_ping = start; next_load = start
    while time.monotonic() - start < a.secs:
        now = time.monotonic()
        if now >= next_ping:
            c = PING_BASE + nping
            sent_t[c] = now; w.write(build_frame(c, c, a.payload)); nping += 1
            next_ping += 1.0 / a.hz
        if a.load and now >= next_load:
            w.write(build_frame(nload, nload, 106)); nload += 1
            next_load += 118.0 / a.load
        time.sleep(0.0005)
time.sleep(2); stop.set(); t.join(timeout=2)
d = sorted((recv_t[c] - sent_t[c]) * 1000 for c in sent_t if c in recv_t)
if not d:
    raise SystemExit(f"load={a.load:4d} n=0 lost={len(sent_t)}: nothing arrived")
lost = len(sent_t) - len(d)
q = lambda p: d[min(len(d) - 1, int(p * len(d)))]
print(f"load={a.load:4d} n={len(d)} lost={lost} min={d[0]:.1f} p50={q(0.5):.1f} p95={q(0.95):.1f} max={d[-1]:.1f} ms")
