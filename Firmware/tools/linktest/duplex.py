#!/usr/bin/env python3
"""Send paced MAVLink v2 frames in both directions at once and count loss on each side."""
import argparse, threading, time, serial
from linktest import build_frame, parse_stream

def side(port, payload, rate, duration, stats, start_evt, baud=57600):
    wire = 12 + payload
    got, sent, buf = [], 0, b""
    with serial.Serial(port, baud, timeout=0) as s:
        s.reset_input_buffer()
        start_evt.wait()
        start = time.time(); nxt = start
        while time.time() - start < duration + 2.5:
            now = time.time()
            if now - start < duration and now >= nxt:
                s.write(build_frame(sent, sent, payload)); sent += 1
                nxt += wire / float(rate)
            d = s.read(4096)
            if d:
                buf, g = parse_stream(buf + d, None)
                got.extend(g)
            else:
                time.sleep(0.001)
    stats[port] = dict(sent=sent, got=got, wire=wire)

ap = argparse.ArgumentParser()
ap.add_argument("--a", default="/dev/ttyUSB0"); ap.add_argument("--b", default="/dev/ttyUSB1")
ap.add_argument("--payload-a", type=int, default=106); ap.add_argument("--payload-b", type=int, default=106)
ap.add_argument("--rate-a", type=int, default=1000); ap.add_argument("--rate-b", type=int, default=1000)
ap.add_argument("--secs", type=float, default=20)
a = ap.parse_args()
stats, go = {}, threading.Event()
ts = [threading.Thread(target=side, args=(a.a, a.payload_a, a.rate_a, a.secs, stats, go)),
      threading.Thread(target=side, args=(a.b, a.payload_b, a.rate_b, a.secs, stats, go))]
for t in ts: t.start()
time.sleep(0.5); go.set()
for t in ts: t.join()
for src, dst, rate in ((a.a, a.b, a.rate_a), (a.b, a.a, a.rate_b)):
    got = set(stats[dst]["got"])
    # everything sent counts; the drain at the end gives the tail time to arrive
    exp = list(range(stats[src]["sent"]))
    lost = [c for c in exp if c not in got]
    w = stats[src]["wire"]
    print(f"{src[-4:]}->{dst[-4:]} wire={w:3d} rate={rate:5d} scored={len(exp):5d} lost={len(lost):4d} "
          f"loss={100.0*len(lost)/max(1,len(exp)):6.2f}% thru={(len(exp)-len(lost))*w/a.secs:6.0f} B/s")
