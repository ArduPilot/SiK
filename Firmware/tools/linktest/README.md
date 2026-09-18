# Link test tools

Scripts to measure a pair of SiK radios directly, both connected to one PC over USB, with nothing else in the path.
They send well-formed MAVLink v2 frames, so the radios' MAVLink framing and RADIO_STATUS injection behave as with a real autopilot.
Needs Python 3 with pyserial.

| Script | What it does |
|---|---|
| `sikat.py PORT CMD...` | Run AT or RT commands, e.g. `sikat.py /dev/ttyUSB0 ATI5 ATI7` |
| `linktest.py` | Send frames of a given size and rate one way and count loss; `--burst N` sends N back to back, `--heartbeat` makes the receiving radio inject RADIO_STATUS |
| `duplex.py` | Send in both directions at once and count loss on each side |
| `latency.py` | One-way delay of small frames, optionally with background load |
| `tdmcount.py` | A `linktest.py` run plus the ATI8 counters of both radios for that run |
| `benchmark.py` | Flash several builds in turn and run the comparison series for one ECC and AIR_SPEED setting |

`tdmcount.py` needs firmware built with the ATI8 counters on both radios: `make clean~radio~<board>` and then `make TDM_COUNTERS=1 build~radio~<board>`.

Both radios must share NETID, AIR_SPEED and ECC. Keep other radios with the same settings switched off while measuring.
The delay from `latency.py` includes the USB serial adapters, about 16 ms with the default FTDI latency timer.
