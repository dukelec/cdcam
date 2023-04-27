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

module cam_csr
    #(
        parameter VERSION = 8'h11
    )(
        input               clk,
        input               reset_n,
        output              irq,
        input               chip_select,

        input       [4:0]   csr_address,
        input               csr_read,
        output reg  [7:0]   csr_readdata,
        input               csr_write,
        input       [7:0]   csr_writedata,

        output reg  [7:0]   rx_ram_rd_addr,
        output reg          rx_ram_rd_done,
        output reg          rx_clean_all,
        input       [7:0]   rx_ram_rd_byte,
        input      [15:0]   rx_ram_rd_flags,
        input               rx_ram_lost,
        input               rx_pending,
        
        output reg  [7:0]   pkt_size
    );

localparam
    REG_VERSION         = 'h00,
    REG_SETTING         = 'h02,
    REG_PKT_SIZE        = 'h04,
    REG_INT_FLAG        = 'h10,
    REG_INT_MASK        = 'h11,
    REG_RX              = 'h14,
    REG_RX_CTRL         = 'h16,
    REG_RX_ADDR         = 'h18,
    REG_RX_PAGE_FLAG    = 'h19;

reg chip_select_delayed;
reg [1:0] cmd_addr;
reg rx_lost_flag;

reg [7:0] int_mask;
wire [7:0] int_flag = {4'd0, rx_lost_flag, 1'b0, rx_pending, 1'b0};
reg [7:0] int_flag_snapshot;

assign irq = (int_flag & int_mask) != 0;

always @(posedge clk) begin
    if (!chip_select)
        cmd_addr <= 0;
    else if (csr_write || csr_read)
        cmd_addr <= cmd_addr + 1'd1;
end


always @(*)
    case (csr_address)
        REG_VERSION:
            csr_readdata = VERSION;
        REG_PKT_SIZE:
            csr_readdata = pkt_size;
        REG_INT_FLAG:
            csr_readdata = int_flag_snapshot;
        REG_INT_MASK:
            csr_readdata = int_mask;
        REG_RX:
            csr_readdata = rx_ram_rd_byte;
        REG_RX_ADDR:
            csr_readdata = rx_ram_rd_addr;
        REG_RX_PAGE_FLAG: begin
            csr_readdata = rx_ram_rd_flags;
            if (cmd_addr[1:0] == 0)
                csr_readdata = rx_ram_rd_flags[7:0];
            else
                csr_readdata = rx_ram_rd_flags[15:8];
        end
        default:
            csr_readdata = 0;
    endcase


always @(posedge clk or negedge reset_n)
    if (!reset_n) begin
        pkt_size <= 249; // real size = pkt_size + 1
        rx_lost_flag <= 0;
        int_mask <= 0;
        int_flag_snapshot <= 0;
        rx_ram_rd_addr <= 0;
        rx_ram_rd_done <= 0;
        rx_clean_all <= 0;
    end
    else begin
        rx_ram_rd_done <= 0;
        rx_clean_all <= 0;
        chip_select_delayed <= chip_select;

        if (!chip_select) begin
            int_flag_snapshot <= int_flag;
            
            if (chip_select_delayed && rx_ram_rd_addr != 0) begin
                rx_ram_rd_done <= 1;
                rx_ram_rd_addr <= 0;
            end
        end

        if (csr_read) begin
            if (csr_address == REG_INT_FLAG) begin
                rx_lost_flag <= 0;
            end
            if (csr_address == REG_RX)
                rx_ram_rd_addr <= rx_ram_rd_addr + 1'd1; 
        end

        if (rx_ram_lost)
            rx_lost_flag <= 1;

        if (csr_write)
            case (csr_address)
                REG_INT_MASK:
                    int_mask <= csr_writedata;
                REG_PKT_SIZE:
                    pkt_size <= csr_writedata;
                REG_RX_CTRL: begin
                    if (csr_writedata[4])
                        rx_clean_all <= 1;
                    if (csr_writedata[1])
                        rx_ram_rd_done <= 1;
                    if (csr_writedata[0])
                        rx_ram_rd_addr <= 0;
                end
                REG_RX_ADDR: begin
                    rx_ram_rd_addr <= csr_writedata;
                end
            endcase
    end

endmodule

