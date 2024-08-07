# Constrain clock port clk with a 42-ns requirement (24MHz)
# 12.288: 81ns
# 111.111: 9ns


create_clock -period 25 -name {global_clk} [get_nets {g_clk}]

set_clock_uncertainty -setup -from [get_clocks {global_clk}] -to [get_clocks {global_clk}] 0.200
#set_clock_uncertainty -hold -from [get_clocks {global_clk}] -to [get_clocks {global_clk}] 0.050


create_clock -period 39 -name {spi_clk} [get_nets {sck}]

set_clock_uncertainty -setup -from [get_clocks {spi_clk}] -to [get_clocks {spi_clk}] 0.200
#set_clock_uncertainty -hold -from [get_clocks {spi_clk}] -to [get_clocks {spi_clk}] 0.050

set_output_delay -max 13 -clock spi_clk -clock_fall [get_ports {sdo}]
set_max_delay -to [get_ports {sdo}] 13


create_clock -period 39 -name {cam_pclk} [get_nets {cam_pclk}]

set_clock_uncertainty -setup -from [get_clocks {cam_pclk}] -to [get_clocks {cam_pclk}] 0.200
#set_clock_uncertainty -hold -from [get_clocks {cam_pclk}] -to [get_clocks {cam_pclk}] 0.050


set_clock_groups -asynchronous -group {global_clk} -group {spi_clk}
set_clock_groups -asynchronous -group {global_clk} -group {cam_pclk}


#create_clock -name vclk -period 10
#set_clock_uncertainty -setup 0.3 [get_clocks {vclk}]
#set_input_delay -max 0.4 -clock vclk [get_ports {clk_i}]
#set_output_delay -max 0.4 -clock vclk [get_ports {clk_o}]
