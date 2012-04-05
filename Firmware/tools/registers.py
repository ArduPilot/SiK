#!/usr/bin/env python

import sys

for line in sys.stdin:
    line = line.strip()
    a = line.split('\t')
    sys.stdout.write('\t{')
    for i in range(len(a)-1):
        sys.stdout.write('0x%s,\t' % a[i])
    sys.stdout.write('0x%s},\n' % a[-1])
