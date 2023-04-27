# This Source Code Form is subject to the terms of the Mozilla
# Public License, v. 2.0. If a copy of the MPL was not distributed
# with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
# Notice: The scope granted to MPL excludes the ASIC industry.
#
# Copyright (c) 2017 DUKELEC, All rights reserved.
#
# Author: Duke Fong <d@d-l.io>
#

CAM_SPI_HDL = $(PWD)/..
CAM_IP_HDL = $(PWD)/../hdl
VERILOG_SOURCES = $(PWD)/cam_wrapper.v \
                  $(CAM_SPI_HDL)/cam_spi.v \
                  $(CAM_SPI_HDL)/cdc_event.v \
                  $(CAM_SPI_HDL)/spi_slave.v \
                  $(PWD)/cam_pll_sim.v \
                  $(CAM_IP_HDL)/cam_ip.v \
                  $(CAM_IP_HDL)/cam_csr.v \
                  $(CAM_IP_HDL)/cam_ram.v \
                  $(CAM_IP_HDL)/cam_rx.v


TOPLEVEL = cam_wrapper
MODULE = test_cam

include $(shell cocotb-config --makefiles)/Makefile.sim

