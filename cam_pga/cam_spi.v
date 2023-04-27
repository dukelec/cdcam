/*
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License, v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * Notice: The scope granted to MPL excludes the ASIC industry.
 *
 * Copyright (c) 2017 DUKELEC, All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */


module cam_spi(
    input       clk_i,

    input       sdi,
    output      sdo,
    input       sck,
    input       nss,

    output      cam_mclk,
    input       cam_vsync,
    input       cam_href,
    input       cam_pclk,
    input [7:0] cam_data,

    output      int_n
);

wire clk = clk_i;
wire cam_mclk = clk_i;

reg rst_sim = 0;
always @(posedge clk)
    rst_sim = 1;

cam_pll cam_pll_m(
    .REFERENCECLK(clk),
    .PLLOUTGLOBAL(g_clk),
    //.PLLOUTCORE(g_clk),
    .LOCK(pll_lock),
    .RESET(rst_sim));

reg [1:0] reset_cnt = 0;
wire reset_n = reset_cnt[0];

always @(posedge g_clk) begin
    if (pll_lock && reset_cnt != 2'b11)
        reset_cnt <= reset_cnt + 1'b1;
end

/* not ok
reg reset_n = 0;

always @(posedge g_clk) begin
    if (pll_lock)
        reset_n <= 1;
end
*/

wire [4:0] csr_address;
wire csr_read;
wire [7:0] csr_readdata;
wire csr_write;
wire [7:0] csr_writedata;

wire irq;
assign int_n = reset_n && irq ? 1'b0 : 1'bz;

spi_slave spi_slave_m(
    .clk(g_clk),
    .reset_n(reset_n),
    .chip_select(chip_select),
    
    .csr_address(csr_address),
    .csr_read(csr_read),
    .csr_readdata(csr_readdata),
    .csr_write(csr_write),
    .csr_writedata(csr_writedata),
    
    .sck(sck),
    .nss(nss),
    .sdi(sdi),
    .sdo(sdo)
);

cam_ip cam_ip_m(
    .clk(g_clk),
    .reset_n(reset_n),
    .chip_select(chip_select),
    
    .csr_address(csr_address),
    .csr_read(csr_read),
    .csr_readdata(csr_readdata),
    .csr_write(csr_write),
    .csr_writedata(csr_writedata),
    
    .irq(irq),
    
    .cam_vsync(cam_vsync),
    .cam_href(cam_href),
    .cam_pclk(cam_pclk),
    .cam_data(cam_data)
);

endmodule
