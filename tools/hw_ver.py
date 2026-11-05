#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2026, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <d@d-l.io>

"""CDCAM v4 hardware version tool

Write the hardware version to the efuse USER_DATA (BLOCK3), shown as "HW: vX.Y[.Z]" in the info string after reboot.
The block is rs coded, it takes one write only: the version can not be changed afterwards.

Args:
  --ver VER           # hardware version, default: 4.0
  --read-only | -r    # only show it
  --yes       | -y    # write without asking
  --bin FILE          # only save the block data for espefuse.py, no device needed
  --dev DEV           # specify serial port, default: ttyACM0
  --baud BAUD         # set baudrate, default: 10000000
  --target-addr ADDR  # default: 00:00:fe
  --help    | -h      # this help message
  --verbose | -v      # debug level: verbose
  --debug   | -d      # debug level: debug
  --info    | -i      # debug level: info

Command example:

$ ./hw_ver.py --ver 4.0

Or by espefuse.py through the usb / uart of the esp (download mode):

$ ./hw_ver.py --ver 4.0 --bin hw_ver.bin
$ espefuse.py -p /dev/ttyACM0 burn_block_data BLOCK_USR_DATA hw_ver.bin

"""

BLK_LEN = 32 # USER_DATA, bytes 0..2: major, minor, patch, same as the fw

import sys, os
import re

sys.path.append(os.path.join(os.path.dirname(__file__), './pycdnet'))

from cdnet.utils.log import *
from cdnet.utils.cd_args import CdArgs

args = CdArgs()
local_mac = int(args.get("--local-mac", dft="0x00"), 0)
dev_str = args.get("--dev", dft="ttyACM0")
baud = int(args.get("--baud", dft="10000000"), 0)
target_addr = args.get("--target-addr", dft="00:00:fe")
ver_str = args.get("--ver", dft="4.0")
bin_file = args.get("--bin")
read_only = args.get("--read-only", "-r") != None
yes = args.get("--yes", "-y") != None


if args.get("--help", "-h") != None:
    print(__doc__)
    exit()

m = re.fullmatch(r'v?(\d+)\.(\d+)(?:\.(\d+))?', ver_str)
ver = tuple(int(x or 0) for x in m.groups()) if m else None
# 0.0.0 can not be told from a blank block
if not ver or max(ver) > 255 or ver == (0, 0, 0):
    print(f'invalid version: {ver_str}')
    exit(-1)

def ver_fmt(v):
    return f'v{v[0]}.{v[1]}.{v[2]}' if v[2] else f'v{v[0]}.{v[1]}'

if bin_file:
    with open(bin_file, 'wb') as f:
        f.write(bytes(ver) + bytes(BLK_LEN - 3))
    print(f'{ver_fmt(ver)} saved to {bin_file}, burn it by:')
    print(f'  espefuse.py -p /dev/{dev_str} burn_block_data BLOCK_USR_DATA {bin_file}')
    exit()


if args.get("--verbose", "-v") != None:
    logger_init(logging.VERBOSE)
elif args.get("--debug", "-d") != None:
    logger_init(logging.DEBUG)
elif args.get("--info", "-i") != None:
    logger_init(logging.INFO)

from cdnet.dev.cdbus_serial import CDBusSerial
from cdnet.dispatch import *

dev = CDBusSerial(dev_str, baud=baud)
CDNetIntf(dev, mac=local_mac)
sock = CDNetSocket(('', 0x40))


def info_read():
    sock.clear()
    sock.sendto(b'', (target_addr, 1))
    ret, _ = sock.recvfrom(timeout=1)
    return ret.decode() if ret else None

def efuse_read():
    sock.clear()
    sock.sendto(b'\x40' + bytes([0, BLK_LEN]), (target_addr, 8))
    ret, _ = sock.recvfrom(timeout=1)
    if not ret or ret[0] != 0 or len(ret) != BLK_LEN + 1:
        print('efuse read error')
        exit(-1)
    return ret[1:]

def efuse_write(dat):
    sock.clear()
    sock.sendto(b'\x60\x00' + dat, (target_addr, 8))
    ret, _ = sock.recvfrom(timeout=2)
    return ret != None and ret[0] == 0

def show(blk):
    cur = tuple(blk[:3]) if any(blk[:3]) else None # same rules as get_hw_ver_str() in the fw
    print(f'  USER_DATA: {blk.hex(" ")}')
    print(f'current: {ver_fmt(cur) if cur else "none"}')
    return cur, blk == bytes(BLK_LEN)


info = info_read()
if not info:
    print(f'no reply from {target_addr}')
    exit(-1)
print(f'info: {info}')
if '(bl)' in info:
    print('the bl can not write it, start the app first')
    exit(-1)

cur, blank = show(efuse_read())
if read_only:
    exit()
if cur == ver:
    print(f'already {ver_fmt(ver)}, nothing to write')
    exit()
if not blank:
    print('the block is written already, it can not be written again')
    exit(-1)

if not yes:
    q = f'write {ver_fmt(ver)} to the efuse, it can not be changed afterwards, continue? [y/N] '
    if input(q).strip().lower() != 'y':
        exit()

ok = efuse_write(bytes(ver))
cur, _ = show(efuse_read())
if not ok or cur != ver:
    print('write failed')
    exit(-1)
print('done, reboot to show it in the info string')
