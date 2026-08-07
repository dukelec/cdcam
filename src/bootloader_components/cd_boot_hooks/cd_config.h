/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CD_CONFIG_H__
#define __CD_CONFIG_H__

#define CDCTL_OSC_CLK       10000000UL // 10MHz

#define CD_FRAME_SIZE       256
#define CDN_MAX_PAYLOAD     251

#define CD_DEBUG
//#define CD_VERBOSE
//#define CD_LIST_DEBUG

//#define CD_SMP
//#define CD_LIST_IT
//#define CD_IRQ_SAFE


// cross-reset flag (survives software reset, cleared on power-on), keep same in app.
// LP_STORE0 is unused by the rom / idf. values: (0xcdcd0000 | do_reboot)
#define BL_ARGS_REG         LP_SYSTEM_REG_LP_STORE0_REG
#define BL_ARGS_KEEP        0xcdcd0001  // reboot and stay in bootloader
#define BL_ARGS_APP         0xcdcd0002  // reboot and boot the app directly


#define d_printf(fmt, ...)  esp_rom_printf(fmt, ##__VA_ARGS__)

#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_rom_spiflash.h"
#include "soc/soc.h"
#include "soc/lp_system_reg.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "hal/gpio_hal.h"
#include "hal/clk_tree_ll.h"
#include "hal/wdt_hal.h"
#include "hal/efuse_ll.h"
#include "hal/gpio_ll.h"
#include "cd_debug.h"
#include "cdctl_spi_wr.h"

#endif
