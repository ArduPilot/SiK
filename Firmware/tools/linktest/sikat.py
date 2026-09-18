#!/usr/bin/env python3
"""Run AT/RT commands against a SiK radio on a given port."""
import serial, sys, time

def at(port, cmds, baud=57600, quiet=False):
    with serial.Serial(port, baud, timeout=0.5) as s:
        time.sleep(1.2)
        s.reset_input_buffer()
        s.write(b"+++"); s.flush()
        time.sleep(1.5)
        if b"OK" not in s.read(4096):
            return None
        out = {}
        for c in cmds:
            s.reset_input_buffer()
            s.write(c.encode() + b"\r\n"); s.flush()
            time.sleep(1.0)
            out[c] = s.read(8192).decode("utf-8", "replace")
            if not quiet:
                print(f"--- {c}\n{out[c].strip()}")
        s.write(b"ATO\r\n"); s.flush()
        return out

if __name__ == "__main__":
    at(sys.argv[1], sys.argv[2:])
