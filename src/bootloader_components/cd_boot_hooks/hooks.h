/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __HOOKS_H__
#define __HOOKS_H__

#include "cd_utils.h"
#include "cdctl.h"

#define BL_SW_VER           "v1.3"

// cross-reset flag (survives software reset, cleared on power-on), keep same in app.
// LP_STORE0 is unused by the rom / idf. values: (0xcdcd0000 | do_reboot)
#define BL_ARGS_REG         LP_SYSTEM_REG_LP_STORE0_REG
#define BL_ARGS_KEEP        0xcdcd0001  // reboot and stay in bootloader
#define BL_ARGS_APP         0xcdcd0002  // reboot and boot the app directly


#define FRAME_MAX           8

// keep same as main.h
#define BL_MCO_PIN          6
#define BL_CD_CS_PIN        7
#define BL_CD_MOSI_PIN      8
#define BL_CD_SCK_PIN       9
#define BL_CD_MISO_PIN      10
#define BL_LED_PIN          14         // green led (LED_G_PIN)

// keep same as cd_main.h
#define APP_CONF_ADDR       0x003ff000 // last page
#define APP_CONF_VER        0x0100

// phase 1: low-speed (115200) window, slow blink;
// phase 2: switch to user baud, fast blink; then boot app
#define BL_PHASE1_MS        1000       // 0 .. 1s at 115200
#define BL_PHASE2_MS        2000       // 1s .. 2s at user baud, then boot app
#define BL_BLINK_SLOW_MS    200        // led toggle interval, phase 1
#define BL_BLINK_FAST_MS    100        // led toggle interval, phase 2
#define BL_FLASH_PROTECT    0x10000    // refuse erase / write below this address


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;      // 1: reboot chip, 2: exit bl, boot app
    bool            keep_in_bl;
    bool            save_conf;

    uint8_t         _reserved01;
    cdctl_cfg_t     bus_cfg;
    uint8_t         dbg_en;
} csa_sm_t;


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;      // 1: reboot chip, 2: exit bl, boot app
    bool            keep_in_bl;
    bool            save_conf;

    uint8_t         _reserved01;
    cdctl_cfg_t     bus_cfg;
    uint8_t         dbg_en;
#define         _end_common _reserved3

    uint8_t         _reserved3[987];

#define _end_save _reserved4    // offset: 1k
    uint8_t         _reserved4;
} csa_t;


extern csa_t csa;
extern const csa_sm_t csa_dft;

extern list_head_t frame_free_head;
extern cdctl_dev_t r_dev;


int flash_cal_crc(uint32_t src_addr, uint32_t len, uint16_t *crc);
int flash_move(uint32_t src_addr, uint32_t dst_addr, uint32_t len);
int flash_read(uint32_t addr, uint8_t *out, uint32_t len);
int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);
void load_conf(void);
int save_conf(void);

void init_info_str(void);
void serial_cmd_dispatch(void);

#endif
