#!/usr/bin/env python

import sys
import re

regex = re.compile("^([0-9A-Fa-f]{2}\s){3}([0-9A-Fa-f]{2}\s?)*(#.*)?$")

bytes = []

for line in sys.stdin:
    atoms = re.split("\s+", line)
    for atom in atoms:
        if len(atom) == 0 or atom == '#':
            break
        bytes.append(int(atom, 16))

chunks = [bytes[i:i+8] for i in range(0, len(bytes), 8)]

print("#include <cstdint>")
print("const uint8_t eeprom_data[] = {")
for chunk in chunks:
    s = ", ".join(['0x{:02x}'.format(x) for x in chunk]) 
    print("    {},".format(s))
print("};")


