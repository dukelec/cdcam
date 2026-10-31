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
#include "cdctl_it.h"
#include "camctl_it.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0200

#define FRAME_MAX           110
#define PACKET_MAX          6
#define CDN_MAX_PAYLOAD     251


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;      // 1: reboot to bl, 2: reboot to app
    bool            _reserved00;    // keep_in_bl for bl
    bool            save_conf;

    uint8_t         _reserved01;
    cdctl_cfg_t     bus_cfg;
    uint8_t         dbg_en;
    uint8_t         _reserved02[6];
    #define         _end_common cam_dst

    cdn_sockaddr_t  cam_dst;
    uint8_t         _reserved1[4];  // roi_x, roi_y on cdcam v4
    uint16_t        width;          // at the address of roi_w / roi_h on cdcam v4
    uint16_t        height;
    uint8_t         _reserved2;     // roi_scale on cdcam v4
    uint8_t         skip;           // report every (skip+1) frames in continuous mode

    uint8_t         _reserved3[6];
    uint8_t         manual;
    uint16_t        exposure;
    uint8_t         agc;
    uint8_t         _reserved4[24];

    // end of flash
    #define         _end_save capture_ctrl

    uint8_t         capture_ctrl;   // 0x80: cancel, 1: one-shot, 8: continuous (2: capture-and-wait: v4 only)
    uint8_t         capture_state;  // 0: idle, 1: one-shot, 8: continuous
    uint8_t         _reserved5[8];
    uint8_t         led_en;

} csa_t; // config status area

// register address map shared with the cdcam v4 (github master) firmware
_Static_assert(offsetof(csa_t, bus_cfg.mac) == 0x000c, "CSA mac offset");
_Static_assert(offsetof(csa_t, bus_cfg.mode) == 0x001a, "CSA bus mode offset");
_Static_assert(offsetof(csa_t, bus_cfg.idle_wait_len) == 0x001b, "CSA idle_wait_len offset");
_Static_assert(offsetof(csa_t, dbg_en) == 0x0024, "CSA dbg_en offset");
_Static_assert(offsetof(csa_t, cam_dst) == 0x002c, "CSA cam_dst offset");
_Static_assert(offsetof(csa_t, width) == 0x0036, "CSA width offset");
_Static_assert(offsetof(csa_t, height) == 0x0038, "CSA height offset");
_Static_assert(offsetof(csa_t, skip) == 0x003b, "CSA skip offset");
_Static_assert(offsetof(csa_t, manual) == 0x0042, "CSA manual offset");
_Static_assert(offsetof(csa_t, exposure) == 0x0044, "CSA exposure offset");
_Static_assert(offsetof(csa_t, capture_ctrl) == 0x005f, "CSA capture_ctrl offset");
_Static_assert(offsetof(csa_t, capture_state) == 0x0060, "CSA capture_state offset");
_Static_assert(offsetof(csa_t, led_en) == 0x0069, "CSA led_en offset");


typedef uint8_t (*hook_func_t)(uint16_t sub_offset, uint8_t len, uint8_t *dat);

typedef struct {
    regr_t          range;
    hook_func_t     before;
    hook_func_t     after;
} csa_hook_t;


extern csa_t csa;
extern const csa_t csa_dft;

extern regr_t csa_w_allow[]; // writable list
extern int csa_w_allow_num;

extern csa_hook_t csa_w_hook[];
extern int csa_w_hook_num;
extern csa_hook_t csa_r_hook[];
extern int csa_r_hook_num;

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

void comm_service_init(void);
void comm_service_poll(void);

uint8_t cam_cfg_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat);
void app_cam_init(void);
void app_cam_poll(void);
void pga_config(void);

extern gpio_t led_r;
extern gpio_t led_g;
extern gpio_t pga_rst;
extern gpio_t pga_cs;
extern cdn_ns_t dft_ns;
extern cdctl_dev_t r_dev;
extern camctl_dev_t cam_dev;
extern spi_t pga_spi;
extern list_head_t frame_free_head;
extern cd_frame_t frame_alloc[];

extern uint32_t end; // end of bss

#endif
