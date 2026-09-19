#!/usr/bin/env python3
"""
Characterise a SiK radio link directly, with nothing else in the path.

Sends well-formed MAVLink v2 FILE_TRANSFER_PROTOCOL frames of a chosen payload
size at a chosen byte rate out of one radio, and counts sequence gaps arriving
out of the other. Payload size and offered rate are set independently, which is
the thing that is awkward to do through PX4 and MAVSDK.
"""
import argparse, serial, struct, sys, threading, time

MAGIC = 0xFD
MSGID = 110          # FILE_TRANSFER_PROTOCOL
CRC_EXTRA = 84

def crc16_mcrf4xx(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def build_frame(seq, counter, payload_len):
    payload = struct.pack("<I", counter) + bytes((counter + i) & 0xFF for i in range(payload_len - 4))
    hdr = struct.pack("<BBBBBB", payload_len, 0, 0, seq & 0xFF, 1, 1) + \
          struct.pack("<I", MSGID)[:3]
    crc = crc16_mcrf4xx(hdr + payload)
    crc = crc16_mcrf4xx(bytes([CRC_EXTRA]), crc)
    return bytes([MAGIC]) + hdr + payload + struct.pack("<H", crc)

def build_heartbeat(seq):
    """A GCS HEARTBEAT (v2). A SiK radio that forwards one starts injecting RADIO_STATUS."""
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 0, 3)
    hdr = struct.pack("<BBBBBB", len(payload), 0, 0, seq & 0xFF, 255, 190) + struct.pack("<I", 0)[:3]
    crc = crc16_mcrf4xx(hdr + payload)
    crc = crc16_mcrf4xx(bytes([50]), crc)
    return bytes([MAGIC]) + hdr + payload + struct.pack("<H", crc)

def parse_stream(buf, seen):
    """Pull complete v2 frames out of buf; return (remaining_buf, [counters])."""
    out = []
    while True:
        i = buf.find(bytes([MAGIC]))
        if i < 0:
            return b"", out
        if len(buf) < i + 12:
            return buf[i:], out
        plen = buf[i + 1]
        total = 12 + plen
        if len(buf) < i + total:
            return buf[i:], out
        frame = buf[i:i + total]
        payload = frame[10:10 + plen]
        crc = crc16_mcrf4xx(frame[1:10 + plen])
        crc = crc16_mcrf4xx(bytes([CRC_EXTRA]), crc)
        if struct.unpack("<H", frame[10 + plen:total])[0] == crc and plen >= 4:
            out.append(struct.unpack("<I", payload[:4])[0])
            buf = buf[i + total:]
        else:
            buf = buf[i + 1:]

def run(tx_port, rx_port, payload_len, rate_bps, duration, baud=57600,
        big_every=0, big_payload=254, reverse_rate=0, burst=1, heartbeat=False):
    wire = 12 + payload_len
    interval = wire / float(rate_bps)
    received, stop = [], threading.Event()

    def reader():
        with serial.Serial(rx_port, baud, timeout=0.2) as r:
            r.reset_input_buffer()
            buf = b""
            rev_n = 0
            rev_len = 20                       # small, like a heartbeat or ack
            rev_interval = (12 + rev_len) / float(reverse_rate) if reverse_rate else 0
            rev_next = time.time() + rev_interval
            hb_n, hb_next = 0, time.time()
            while not stop.is_set():
                d = r.read(4096)
                if d:
                    buf, got = parse_stream(buf + d, None)
                    received.extend(got)
                if heartbeat and time.time() >= hb_next:
                    # like a GCS: makes the receiving radio inject RADIO_STATUS
                    r.write(build_heartbeat(hb_n)); r.flush()
                    hb_n += 1
                    hb_next += 1.0
                if reverse_rate and time.time() >= rev_next:
                    # load the other direction: the radios are half duplex, so
                    # this competes for transmit windows with the main stream
                    r.write(build_frame(rev_n, rev_n, rev_len)); r.flush()
                    rev_n += 1
                    rev_next += rev_interval

    t = threading.Thread(target=reader, daemon=True); t.start()
    time.sleep(0.4)

    sent = 0
    with serial.Serial(tx_port, baud, timeout=0.2) as w:
        start = time.time()
        nxt = start
        while time.time() - start < duration:
            # send a group back to back, the way a paced burst actually leaves
            for _ in range(max(1, burst)):
                if time.time() - start >= duration:
                    break
                this_len = big_payload if (big_every and sent % big_every == 0) else payload_len
                w.write(build_frame(sent, sent, this_len)); w.flush()
                sent += 1
                nxt += (12 + this_len) / float(rate_bps)
            slp = nxt - time.time()
            if slp > 0:
                time.sleep(slp)
    time.sleep(2.5)                      # let the tail drain
    stop.set(); t.join(timeout=2)

    got = set(received)
    # everything sent counts; the drain above gives the tail time to arrive
    expected = list(range(sent))
    lost = [c for c in expected if c not in got]
    lossp = 100.0 * len(lost) / len(expected) if expected else float("nan")
    return dict(payload=payload_len, wire=wire, rate=rate_bps, sent=sent,
                scored=len(expected), recv=len(expected) - len(lost),
                lost=len(lost), loss=lossp,
                thru=(len(expected) - len(lost)) * wire / duration)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tx", default="/dev/ttyUSB1")
    ap.add_argument("--rx", default="/dev/ttyUSB0")
    ap.add_argument("--payload", type=int, default=100)
    ap.add_argument("--rate", type=int, default=1000)
    ap.add_argument("--secs", type=float, default=15)
    ap.add_argument("--big-every", type=int, default=0,
                    help="every Nth packet uses --big-payload instead")
    ap.add_argument("--big-payload", type=int, default=254)
    ap.add_argument("--reverse-rate", type=int, default=0,
                    help="B/s of small frames sent back the other way")
    ap.add_argument("--heartbeat", action="store_true",
                    help="send a GCS heartbeat once a second from the receiving side")
    ap.add_argument("--burst", type=int, default=1,
                    help="packets sent back to back before pausing")
    a = ap.parse_args()
    r = run(a.tx, a.rx, a.payload, a.rate, a.secs, big_every=a.big_every, big_payload=a.big_payload, reverse_rate=a.reverse_rate, burst=a.burst, heartbeat=a.heartbeat)
    print(f"payload={r['payload']:3d} wire={r['wire']:3d} rate={r['rate']:5d} "
          f"scored={r['scored']:4d} lost={r['lost']:4d} loss={r['loss']:6.2f}% "
          f"thru={r['thru']:6.0f} B/s")
