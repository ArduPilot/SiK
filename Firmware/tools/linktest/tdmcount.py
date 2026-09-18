#!/usr/bin/env python3
"""Run linktest.py between two radios and show the ATI8 TDM counters of both for the run.

Usage: tdmcount.py LINKTEST_ARGS...  (ports from SIK_TX / SIK_RX, default ttyUSB0 / ttyUSB1)
Needs firmware with the ATI8 counters on both radios.
"""
import os, subprocess, sys
from sikat import at
# order of the values ATI8 prints
NAMES = "tx_data tx_bonus tx_yield tx_stats rx_data rx_bonus rx_yield rx_granted rx_stats rx_bonus_late hop_abort".split()
def counters(port):
    out = at(port, ["ATI8", "ATI7"], quiet=True)
    nums = [int(x) for x in out["ATI8"].split()[1:1 + len(NAMES)]]
    return dict(zip(NAMES, nums)), out["ATI7"].split("txe=")[1].split(" temp")[0]
args = sys.argv[1:]
TX = os.environ.get("SIK_TX", "/dev/ttyUSB0")
RX = os.environ.get("SIK_RX", "/dev/ttyUSB1")
counters(TX); counters(RX)
r = subprocess.run(["python3", os.path.join(os.path.dirname(os.path.abspath(__file__)), "linktest.py"), "--tx", TX, "--rx", RX, *args], capture_output=True, text=True).stdout.strip().splitlines()[-1]
(t, te), (x, xe) = counters(TX), counters(RX)
print(r)
print(f"  sender  : data={t['tx_data']} bonus={t['tx_bonus']} yield={t['tx_yield']} stats={t['tx_stats']} | got yield={t['rx_yield']} granted={t['rx_granted']}  [{te}]")
print(f"  receiver: data={x['rx_data']} bonus={x['rx_bonus']} late={x['rx_bonus_late']} hop_abort={x['hop_abort']} | sent yield={x['tx_yield']} stats={x['tx_stats']} data={x['tx_data']}  [{xe}]")
