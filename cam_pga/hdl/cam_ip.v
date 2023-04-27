/*
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License, v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * Notice: The scope granted to MPL excludes the ASIC industry.
 *
 * Copyright (c) 2017 DUKELEC, All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 *
 * This file is the top module of CAM IP.
 */

module cam_ip(
        input               clk,
        input               reset_n,
        input               chip_select, // reduce ram_rx power consumption
        output              irq,

        input       [4:0]   csr_address,
        input               csr_read,
        output      [7:0]   csr_readdata,
        input               csr_write,
        input       [7:0]   csr_writedata,

        // cam
        input               cam_vsync,
        input               cam_href,
        input               cam_pclk,
        input       [7:0]   cam_data
    );

wire [7:0] rx_ram_rd_addr;
wire rx_ram_rd_done;
wire rx_clean_all;
wire [7:0] rx_ram_rd_byte;
wire [15:0] rx_ram_rd_flags;

wire [7:0] rx_ram_wr_byte;
wire [7:0] rx_ram_wr_addr;
wire rx_ram_wr_en;
wire rx_ram_switch;
wire [15:0] rx_ram_wr_flags;

wire [7:0] pkt_size;


cam_csr cd_csr_m(
    .clk(clk),
    .reset_n(reset_n),
    .irq(irq),
    .chip_select(chip_select),

    .csr_address(csr_address),
    .csr_read(csr_read),
    .csr_readdata(csr_readdata),
    .csr_write(csr_write),
    .csr_writedata(csr_writedata),

    .rx_ram_rd_addr(rx_ram_rd_addr),
    .rx_ram_rd_done(rx_ram_rd_done),
    .rx_clean_all(rx_clean_all),
    .rx_ram_rd_byte(rx_ram_rd_byte),
    .rx_ram_rd_flags(rx_ram_rd_flags),
    .rx_ram_lost(rx_ram_lost),
    .rx_pending(rx_pending),
    
    .pkt_size(pkt_size)
);


cd_ram #(.N_WIDTH(3)) cd_ram_rx_m(
    .clk(clk),
    .reset_n(reset_n),

    .rd_byte(rx_ram_rd_byte),
    .rd_addr(rx_ram_rd_addr),
    .rd_en(chip_select),
    .rd_done(rx_ram_rd_done),
    .rd_done_all(rx_clean_all),
    .unread(rx_pending),

    .wr_byte(rx_ram_wr_byte),
    .wr_addr(rx_ram_wr_addr),
    .wr_en(rx_ram_wr_en),

    .switch(rx_ram_switch),
    .wr_flags(rx_ram_wr_flags),
    .rd_flags(rx_ram_rd_flags),
    .switch_fail(rx_ram_lost)
);

cam_rx cam_rx_m(
    .clk(clk),
    .reset_n(reset_n),

    .abort(rx_clean_all),
    .pkt_size(pkt_size),

    .cam_vsync(cam_vsync),
    .cam_href(cam_href),
    .cam_pclk(cam_pclk),
    .cam_data(cam_data),

    .ram_wr_byte(rx_ram_wr_byte),
    .ram_wr_addr(rx_ram_wr_addr),
    .ram_wr_en(rx_ram_wr_en),
    .ram_wr_flags(rx_ram_wr_flags),
    .ram_switch(rx_ram_switch)
);

endmodule

