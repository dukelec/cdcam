/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cdnet_core.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0200

#define FRAME_MAX           30
#define PACKET_MAX          30
#define CDN_MAX_PAYLOAD     251


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: load from flash
    uint8_t         do_reboot;      // 1: reboot to bl, 2: reboot to app
    bool            keep_in_bl;
    bool            save_conf;

    uint8_t         _reserved01;
    cdctl_cfg_t     bus_cfg;
    uint8_t         dbg_en;

    uint8_t         _reserved2[987]; // covers the areas in the app csa that need to be saved

    #define         _end_save _reserved3 // offset: 1 KiB
    uint8_t         _reserved3;

} csa_t; // config status area

_Static_assert(offsetof(csa_t, bus_cfg.mac) == 0x000c, "CSA mac offset");
_Static_assert(offsetof(csa_t, dbg_en) == 0x0024, "CSA dbg_en offset");
_Static_assert(offsetof(csa_t, _end_save) == 0x0400, "CSA saved config size");

extern csa_t csa;
extern const csa_t csa_dft;

extern cdctl_dev_t r_dev;
extern list_head_t frame_free_head;

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

void app_main(void);
void load_conf(void);
int save_conf(void);

void comm_service_init(void);
void comm_service_poll(void);

extern gpio_t led_r;
extern gpio_t led_g;
extern cdn_ns_t dft_ns;
extern uint32_t *bl_args;

#endif
