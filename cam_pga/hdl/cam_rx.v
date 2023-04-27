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

module cam_rx(
        input               clk,
        input               reset_n,

        // cd_csr
        input               abort,
        input       [7:0]   pkt_size,

        // cam
        input               cam_vsync,
        input               cam_href,
        input               cam_pclk,
        input       [7:0]   cam_data,

        // pp_ram
        output      [7:0]   ram_wr_byte,
        output reg  [7:0]   ram_wr_addr,
        output reg          ram_wr_en,
        output reg [15:0]   ram_wr_flags,
        output reg          ram_switch
    );

reg [7:0] byte_cnt, byte_cnt_pend;
reg [1:0] type_, type_pend;
reg [3:0] cnt, cnt_pend;
reg [7:0] pend_addr, pend_addr_bk;
reg has_pend, pend_wr, pend_end;

reg vsync, _vsync, vsync_d, vsync_d_bk;
reg csync, _csync, csync_d, csync_d_bk;
reg [7:0] cdata, _cdata, cdata_d, cdata_pend, cdata_pend_bk;
assign ram_wr_byte = cdata_pend_bk;


always @(posedge clk) begin
    vsync_d <= _vsync; _vsync <= vsync;
    csync_d <= _csync; _csync <= csync;
    cdata_d <= _cdata; _cdata <= cdata;
end


always @(posedge cam_pclk or negedge reset_n)
    if (!reset_n) begin
        csync <= 0;
    end
    else begin
        vsync <= cam_vsync;
        if (cam_vsync & cam_href) begin
            cdata <= cam_data;
            csync <= !csync;
        end
    end


always @(posedge clk or negedge reset_n)
    if (!reset_n) begin
        byte_cnt <= 0; byte_cnt_pend <= 0;
        cnt <= 0; cnt_pend <= 0;
        type_ <= 0; type_pend <= 0; // type: error
        has_pend <= 0; pend_wr <= 0; pend_end <= 0;
        cdata_pend <= 0; cdata_pend_bk <= 0;
        pend_addr <= 0; pend_addr_bk <= 0;
    end
    else begin
        pend_wr <= 0;
        pend_end <= 0;
        
        if (csync_d != csync_d_bk) begin
            cdata_pend <= cdata_d;
            pend_addr <= byte_cnt;
            byte_cnt <= byte_cnt + 1'd1;
            has_pend <= 1;
            
            if (has_pend) begin
                pend_addr_bk <= pend_addr;
                cdata_pend_bk <= cdata_pend;
                pend_wr <= 1;
                if (byte_cnt == 0)
                    pend_end <= 1;
            end
            
            if (byte_cnt >= pkt_size) begin
                type_pend <= type_;
                cnt_pend <= cnt;
                byte_cnt_pend <= byte_cnt + 1;
                
                byte_cnt <= 0;
                cnt <= cnt + 1'd1;
                type_ <= 2'b10; // type: more
            end
        end
        else if (vsync_d == 0 && vsync_d_bk != 0) begin
            pend_addr_bk <= pend_addr;
            cdata_pend_bk <= cdata_pend;
            pend_wr <= 1;
            type_pend <= 2'b11;
            pend_end <= 1;
            has_pend <= 0;
            if (byte_cnt != 0) begin
                byte_cnt_pend <= byte_cnt;
                cnt_pend <= cnt;
            end
            
            byte_cnt <= 0;
            cnt <= 0;
            type_ <= 2'b01; // type: first
        end
        
        if (abort) begin
            byte_cnt <= 0;
            cnt <= 0;
            type_ <= 0; // type: error
            has_pend <= 0;
        end
        
        vsync_d_bk <= vsync_d;
        csync_d_bk <= csync_d;
    end


always @(posedge clk or negedge reset_n)
    if (!reset_n) begin
        ram_wr_addr <= 0;
        ram_wr_en <= 0;
        ram_wr_flags <= 0;
        ram_switch <= 0;
    end
    else begin
        ram_switch <= 0;
        ram_wr_en <= 0;
        
        if (pend_wr) begin
            ram_wr_addr <= pend_addr_bk;
            ram_wr_en <= 1;
        end
        
        if (pend_end) begin
            ram_wr_flags <= {2'b00, type_pend, cnt_pend, byte_cnt_pend};
            ram_switch <= 1;
        end
        
        if (abort) begin
            ram_switch <= 0;
        end
    end

endmodule

