/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cdnet_dispatch.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl_it.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0101

#define FRAME_MAX           10
#define PACKET_MAX          60


typedef enum {
    LED_POWERON = 0,
    LED_WARN,
    LED_ERROR
} led_state_t;

typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint16_t        conf_ver;
    bool            conf_from;  // 0: default, 1: load from flash
    bool            do_reboot;
    bool            _reserved;
    bool            save_conf;

    uint8_t         bus_net;
    cdctl_cfg_t     bus_cfg;
    bool            dbg_en;
    cdn_sockaddr_t  dbg_dst;

    uint8_t         _save_end;

} csa_t; // config status area


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


void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

void common_service_init(void);
void common_service_routine(void);

void set_led_state(led_state_t state);

uint8_t cam_w_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat);
void app_cam_init(void);
void app_cam_routine(void);

extern gpio_t led_r;
extern gpio_t led_g;
extern cdn_ns_t dft_ns;
extern list_head_t frame_free_head;

#endif