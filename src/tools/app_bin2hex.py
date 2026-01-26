#!/usr/bin/env python3
#
# Software License Agreement (MIT License)
#
# Author: Duke Fong <d@d-l.io>

import struct
import sys
from PyCRC.CRC16 import CRC16
from intelhex import IntelHex

OTA_ADDR = 0x120000


def modbus_crc(frame):
    return CRC16(modbus_flag=True).calculate(frame)


def bin_to_hex_with_size_header(bin_file, hex_file):
    with open(bin_file, 'rb') as f:
        bin_data = f.read()

    size = len(bin_data)
    print(f"Binary size: {size} bytes")
    bin_data += modbus_crc(bin_data).to_bytes(2, byteorder='little')

    size_header = struct.pack('<I', size | 0xcd000000)
    full_data = size_header + bin_data

    ih = IntelHex()
    ih.puts(OTA_ADDR, full_data)
    ih.write_hex_file(hex_file)
    print(f"Intel HEX file written to {hex_file}")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("usage: ./script.py input.bin")
    else:
        bin_to_hex_with_size_header(sys.argv[1], f'{sys.argv[1][:-4]}.hex')

