# This Source Code Form is subject to the terms of the Mozilla
# Public License, v. 2.0. If a copy of the MPL was not distributed
# with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
# Notice: The scope granted to MPL excludes the ASIC industry.
#
# Copyright (c) 2017 DUKELEC, All rights reserved.
#
# Author: Duke Fong <d@d-l.io>
#

import cocotb
from cocotb.binary import BinaryValue
from cocotb.triggers import RisingEdge, FallingEdge, ReadOnly, Timer
from cocotb.clock import Clock


REG_VERSION         = 0x00
REG_SETTING         = 0x02
REG_PKT_SIZE        = 0x04
REG_INT_FLAG        = 0x10
REG_INT_MASK        = 0x11
REG_RX              = 0x14
REG_RX_CTRL         = 0x16
REG_RX_ADDR         = 0x18
REG_RX_PAGE_FLAG    = 0x19

BIT_FLAG_RX_LOST            = 1 << 3
BIT_FLAG_RX_PENDING         = 1 << 1

BIT_RX_RST                  = 1 << 4
BIT_RX_CLR_PENDING          = 1 << 1
BIT_RX_RST_POINTER          = 1 << 0

CLK_FREQ = 40000000
CLK_PERIOD = 1000000000000 / CLK_FREQ

SPI_FREQ = 32000000
SPI_PERIOD = 1000000000000 / SPI_FREQ


async def send_frame(dut):
    with open('cam_data.csv') as f:
        line = f.readline()
        last_time = 0
        data = 0
        while True:
            line = f.readline()
            if not line:
                break
            dat = line.split(', ')
            #time = round(float(dat[0]) * 1000000000000)
            time = round(float(dat[0]) * 50000000000)
            pclk = int(dat[3])
            href = int(dat[1])
            vsync = int(dat[2])
            data = data + 1
            if data >= 256 * 2:
                data = 0
            if last_time != 0:
                await Timer(time - last_time)
            last_time = time
            
            dut.cam_vsync.value = vsync
            dut.cam_href.value = href
            dut.cam_pclk.value = pclk
            dut.cam_data.value = data >> 1


async def spi_rw(dut, w_data = 0):
    r_data = 0
    for i in range(0,8):
        dut.sdi.value = 1 if (w_data & 0x80) else 0
        w_data = w_data << 1
        dut.sck_scl.value = 0
        await Timer(SPI_PERIOD / 2)
        dut.sck_scl.value = 1
        await ReadOnly()
        if dut.sdo_sda.value.binstr != 'z':
            r_data = (r_data << 1) | dut.sdo_sda.value.integer
        else:
            r_data = (r_data << 1) | 0
        await Timer(SPI_PERIOD / 2)
        dut.sck_scl.value = 0
    return r_data

async def spi_read(dut, address, len = 1):
    datas = []
    dut.nss.value = 0
    await Timer(SPI_PERIOD / 2)
    await spi_rw(dut, address)
    await Timer(SPI_PERIOD / 2)
    while len != 0:
        ret_val = await spi_rw(dut)
        datas.append(ret_val)
        await Timer(SPI_PERIOD / 2)
        len -= 1
    dut.nss.value = 1
    await Timer(SPI_PERIOD / 2)
    return datas

async def spi_write(dut, address, datas):
    dut.nss.value = 0
    await Timer(SPI_PERIOD / 2)
    await spi_rw(dut, address | 0x80)
    await Timer(SPI_PERIOD / 2)
    for data in datas:
        await spi_rw(dut, data)
        await Timer(SPI_PERIOD / 2)
    dut.nss.value = 1
    await Timer(SPI_PERIOD / 2)


@cocotb.test()
async def test_cam(dut):
    """
    test_cam
    """
    dut._log.info("test_cam start.")
    dut.nss.value = 1
    dut.sck_scl.value = 0

    cocotb.fork(Clock(dut.clk, CLK_PERIOD).start())
    await Timer(500000) # wait reset

    value = await spi_read(dut, REG_VERSION)
    dut._log.info("REG_VERSION: 0x%02x" % int(value[0]))
    
    await spi_write(dut, REG_INT_MASK, [0x02])
    
    await Timer(15000000)

    cocotb.fork(send_frame(dut));
    
    for i in range(66):#40
        if i == 4:
            await spi_write(dut, REG_RX_CTRL, [0x10])
        await Timer(5000000)
        print("dut.int_n.value.binstr", dut.int_n.value.binstr)
        if dut.int_n.value.binstr == 'z':
            await FallingEdge(dut.int_n)
        value = await spi_read(dut, REG_RX_PAGE_FLAG, 2)
        print(" ".join([("%02x" % x) for x in value]), '-------------', i)
        if (value[1] & 0xf0) != 0x20:
            print("====================================+++++=======")
        value = await spi_read(dut, REG_RX, value[0])
        print(" ".join([("%02x" % x) for x in value]))
        #await spi_write(dut, REG_RX_CTRL, [0x03])
    
    await Timer(12000000000)

    dut._log.info("test_cdctl_bx done.")

