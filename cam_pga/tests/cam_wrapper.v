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

`timescale 1 ns / 1 ps

module cam_wrapper(
        input       clk,

        input       sdi,
        inout       sdo_sda,
        input       sck_scl,
        input       nss,

        output      int_n,

        input       cam_vsync,
        input       cam_href,
        input       cam_pclk,
        input [7:0] cam_data
    );


cam_spi cam_spi_m(
          .clk_i(clk),
          
          .sdi(sdi),
          .sdo(sdo_sda),
          .sck(sck_scl),
          .nss(nss),
          
          .int_n(int_n),
          
          .cam_vsync(cam_vsync),
          .cam_href(cam_href),
          .cam_pclk(cam_pclk),
          .cam_data(cam_data)
      );

initial begin
    $dumpfile("cam.vcd");
    $dumpvars();
end

endmodule
