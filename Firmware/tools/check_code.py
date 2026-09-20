#!/usr/bin/env python3
'''
check the radio code for prototype consistency
Andrew Tridgell, February 2012
'''

import re, glob, sys, os

header_protos = {}
code_protos = {}

hmatch = re.compile(r'^extern\s+.*\s+(\w+)\(.*\)');
cmatch = re.compile(r'^(\w+)\(.*\)');

if len(sys.argv) < 2:
    print("ERROR: board not defined")
    print(sys.argv)
    sys.exit(1)

if len(sys.argv) < 3:
    print("ERROR: xram size not defined")
    print(sys.argv)
    sys.exit(1)

if int(sys.argv[-1]) < 4096:
    print("ERROR: xram invalid")
    sys.exit(1)

board     = sys.argv[1].split('.')[0]
xram_size = int(sys.argv[-1])


def extract_header_functions(h, d):
    '''extract extern functions from a header'''
    f = open(h)
    for line in f:
        line = line.strip()
        line = line.replace('\t', ' ')
        line = line.replace('  ', ' ')
        if line.endswith(';'):
            line = line[:-1]
        m = hmatch.match(line)
        if m:
            d[m.group(1)] = line

def extract_C_functions(c, d):
    '''extract extern functions from a header'''
    f = open(c)
    prevline = ''
    for line in f:
        line = line.rstrip()
        line = line.replace('\t', ' ')
        line = line.replace('  ', ' ')
        m = cmatch.match(line)
        if m and not prevline.startswith('static') and line.find('INTERRUPT') == -1:
            d[m.group(1)] = prevline + ' ' + line
        prevline = line

error_count = 0

def check_xiseg():
    '''check that external RAM has not overflowed'''
    global error_count
    # XSEG holds the uninitialised xdata and XISEG the initialised part, and a
    # firmware can end up with either of them empty, so look at both and take
    # whichever ends higher
    xmatch = re.compile(r'^(XISEG|XSEG)\s*(\w+)\s*(\w+)');
    for map in glob.glob("%s.map"%board):
        f = open(map)
        end = 0
        name = None
        for line in f:
            m = xmatch.match(line)
            if m:
                ofs1 = int(m.group(2),16)
                ofs2 = int(m.group(3),16)
                if ofs1 + ofs2 > end:
                    end = ofs1 + ofs2
                    name = m.group(1)
        if name is None:
            continue
        print(os.popen("tail -n5 %s.mem"%board).read())
        print('%s %s - %u bytes available' % (name, map, xram_size-end))
        if end >= xram_size:
            print('ERROR: %s overflow %u in %s' % (name, end, map))
            error_count += 1


# go through all the headers looking for extern declarations of functions
for h in glob.glob('radio/*.h'):
    extract_header_functions(h, header_protos)

for c in glob.glob('radio/*.c'):
    extract_C_functions(c, code_protos)

for h in header_protos:
    if not h in code_protos:
        print("No code proto for %s: %s\n" % (h, header_protos[h]))
        error_count += 1
        continue
    if header_protos[h] != 'extern ' + code_protos[h]:
        print('Header: %s\nCode:   %s\n' % (header_protos[h], code_protos[h]))
        error_count += 1
            

for c in code_protos:
    if not c in header_protos and not c in ['main', 'putchar', '__at']:
        print("No header proto for %s: %s\n" % (c, code_protos[c]))
        error_count += 1
        continue

check_xiseg()
if error_count:
    print("ERROR: code checked failed with %u errors" % error_count)
    sys.exit(1)
print("Code check OK")

