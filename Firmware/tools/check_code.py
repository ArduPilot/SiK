#!/usr/bin/env python
'''
check the radio code for prototype consistency
Andrew Tridgell, February 2012
'''

import re, glob, sys

header_protos = {}
code_protos = {}

hmatch = re.compile(r'^extern\s+.*\s+(\w+)\(.*\)');
cmatch = re.compile(r'^(\w+)\(.*\)');

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
    '''check that XISEG has not overflowed'''
    global error_count
    xmatch = re.compile(r'^XISEG\s*(\w+)\s*(\w+)');
    for map in glob.glob("obj/*/radio*/*map"):
        f = open(map)
        for line in f:
            m = xmatch.match(line)
            if m:
                ofs1 = int(m.group(1),16)
                ofs2 = int(m.group(2),16)
                if ofs1 + ofs2 >= 4096:
                    print('ERROR: XISEG overflow %u in %s' % (ofs1+ofs2, map))
                    error_count += 1
                else:
                    print('XISEG %s - %u bytes available' % (map, 4096-(ofs1+ofs2)))


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

