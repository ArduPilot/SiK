#!/usr/bin/env python
'''
check the radio code for prototype consistency
Andrew Tridgell, February 2012
'''

import re, glob

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

# go through all the headers looking for extern declarations of functions
for h in glob.glob('radio/*.h'):
    extract_header_functions(h, header_protos)

for c in glob.glob('radio/*.c'):
    extract_C_functions(c, code_protos)

for h in header_protos:
    if not h in code_protos:
        print("No code proto for %s: %s\n" % (h, header_protos[h]))
        continue
    if header_protos[h] != 'extern ' + code_protos[h]:
        print('Header: %s\nCode:   %s\n' % (header_protos[h], code_protos[h]))
            

for c in code_protos:
    if not c in header_protos and not c in ['main', 'putchar', '__at']:
        print("No header proto for %s: %s\n" % (c, code_protos[c]))
        continue
