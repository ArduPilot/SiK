# This script generates radio/radio_446x_conf.h
#
# Silabs' WDS software allows to export a header file containing register values
# for a specific configuration of the radio. For our use we want to be able to
# switch between couple of those configurations. That's where this script comes in.
# It compares a set of headers generated from WDS and compiles all the register
# values into a somewhat compressed format. As part of this format it singles out
# registers whose value does not change between all the considered configurations.

import glob
import re

propbytes = dict()
skip = ['RF_MODEM_RSSI_CONTROL_1', 'RF_MODEM_RSSI_COMP_1',
        'RF_MODEM_CLKGEN_BAND_1']

for fn in glob.glob('*.h'):
    with open(fn, 'r') as f:
        for d in re.finditer('\n#define (RF_[^ ]+) (.+)\n', f.read()):
            if d.group(1) in skip:
                continue
            bytes_ = eval('[' + d.group(2) + ']')
            if bytes_[0] != 0x11:
                continue
            group, off = bytes_[1], bytes_[3]
            # keep MODEM, MODEM_CHFLT, PA, SYNTH
            if group < 0x20 or group >= 0x30:
                continue
            for i, v in enumerate(bytes_[4:]):
                k = group*256 + (off + i)
                if k not in propbytes:
                    propbytes[k] = dict()
                propbytes[k][fn] = v

def print_bytes(bytes_):
    print(", ".join(["0x%02x" % b for b in bytes_]), end="")

def ids_list(indices):
    indices = sorted(indices)
    indices_starts = [k for k in indices if k-1 not in indices]
    ret = []
    for i, k in enumerate(indices_starts):
        last = i == len(indices_starts) - 1
        l = len([k_ for k_ in indices if k_ >= k \
                 and (last or k_ < indices_starts[i+1])])
        while l > 0:
            l_ = min(l, 12)
            ret += [l_, k//256, k%256]
            l -= l_
            k += l_
    return ret + [0]

shared = sorted([k for k in propbytes if len(set(propbytes[k].values())) == 1])

print("/* see tools/build_si446x_table.py */\n")

print("__code static const uint8_t shared_prop_ids[] = {")
print("\t", end="")
print_bytes(ids_list(shared))
print("\n};\n")

print("__code static const uint8_t shared_prop_vals[] = {")
print("\t", end="")
print_bytes([list(propbytes[k].values())[0] for k in shared])
print("\n};\n")

variable = sorted([k for k in propbytes if len(set(propbytes[k].values())) != 1])

print("__code static const uint8_t variable_prop_ids[] = {")
print("\t", end="")
print_bytes(ids_list(variable))
print("\n};\n")

bands = ["433", "868", "915"]
rates = ["2", "4", "8", "16", "19", "24", "32",
         "48", "64", "96", "128", "192", "250"]

for band in bands:
    print("__code static const uint8_t band_%s_prop_vals[%d][%d] = {" \
          % (band, len(rates), len(variable)))
    for rate in rates:
        fn = "%s_%skbps.h" % (band, rate)
        print("\t{", end="")
        print_bytes([propbytes[k][fn] for k in variable])
        print("}%s" % ("" if rate=="250" else ","), end="\n")
    print("};\n")
