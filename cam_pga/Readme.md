CDCAM FPGA Code
=======================================

The SPI interface is the same as: https://github.com/dukelec/cdbus/tree/master/example


## Registers

| Register Name     |  Addr   | Access | Default         | Description                          | Remarks                                              |
|-------------------|---------|--------|-----------------|--------------------------------------|------------------------------------------------------|
| VERSION           |  0x00   | RD     | 0x11            | Hardware version                     |                                                      |
| SETTING           |  0x02   | RD/WR  | 0x10            | Configs                              | Not used                                             |
| PKT_SIZE          |  0x04   | RD/WR  | 249             | Packet size                          |                                                      |
| INT_FLAG          |  0x10   | RD     | n/a             | Status                               |                                                      |
| INT_MASK          |  0x11   | RD/WR  | 0x00            | Interrupt mask                       |                                                      |
| RX                |  0x14   | RD     | n/a             | Read RX page                         | 8 pages as buffer in this design                     |
| RX_CTRL           |  0x16   | WR     | n/a             | RX control                           |                                                      |
| RX_ADDR           |  0x18   | RD/WR  | 0x00            | RX page read pointer                 | Rarely used                                          |
| RX_PAGE_FLAG      |  0x19   | RD     | n/a             | RX page flag                         | Should read two bytes                                |


**PKT_SIZE:**

Max package size is: `PKT_SIZE[7:0] + 1`

The default package size is 250.

**INT_FLAG:**

| FIELD   | DESCRIPTION                                  |
|-------- |----------------------------------------------|
| [3]     | 1: RX lost: no empty page for RX             |
| [1]     | 1: RX page ready for read                    |

Reading this register will automatically clear bit3.

**INT_MASK:**

Output of irq = ((INT_FLAG & INT_MASK) != 0).

**RX_CTRL:**

| FIELD   | DESCRIPTION                 |
|-------- |-----------------------------|
| [4]     | Reset RX block              |
| [1]     | Switch RX page              |
| [0]     | Reset RX page read pointer  |


Read RX page will automatically set bit1 and bit4.

**RX_PAGE_FLAG:**

Read two bytes from this register:

First byte: data length in current RX page.
Second byte:
 - bit[7:6]: always zero.
 - bit[5:4]: types:
   - 00: error;
   - 01: first package of the image;
   - 11: last package of one image;
   - 10: middle packages.
 - bit[3:0]: counter, first package is zero.


## Test

Extract the `cam_data.tar.xz` first.

Install `iverilog` (>= v10) and `cocotb`, goto `tests/` folder, run `make`.
(You can checkout the waveform `cam.vcd` by GTKWave.)


## License
```
This Source Code Form is subject to the terms of the Mozilla
Public License, v. 2.0. If a copy of the MPL was not distributed
with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
Notice: The scope granted to MPL excludes the ASIC industry.
The CDBUS protocol is royalty-free for everyone except chip manufacturers.

Copyright (c) 2017 DUKELEC, All rights reserved.
```

