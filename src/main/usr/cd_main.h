/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CD_MAIN_H__
#define __CD_MAIN_H__

#include "cd_utils.h"
#include "cd_list.h"
#include "cdctl_it.h"
#include "cdbus_uart.h"
#include "modbus_crc.h"

#define APP_CONF_ADDR       0x003ff000 // last page
#define APP_CONF_VER        0x0104

#define FRAME_MAX           500


typedef struct {
    uint8_t  addr[3]; // [type, net, mac] or [type, mh, ml]
    uint16_t port;
} cdn_sockaddr_t;

typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;
    bool            _reserved00;
    bool            save_conf;

    uint8_t         _reserved01;
    cdctl_cfg_t     bus_cfg;
    uint8_t         dbg_en;
    uint8_t         _reserved02[6];
    #define         _end_common cam_dst

    cdn_sockaddr_t  cam_dst;
    uint16_t        width;
    uint16_t        height;
    uint8_t         skip;

    uint8_t         _reserved1[11];
    bool            manual;
    uint16_t        exposure;
    uint8_t         agc;
    uint8_t         _reserved2[24];

    // end of flash
    #define         _end_save capture

    uint8_t         capture;
    uint8_t         _reserved3[9];
    uint8_t         led_en;
    uint8_t         _reserved4[8];
    uint32_t        img_len;
    uint32_t        img_read[2];    // ofs, len
    uint32_t        img_read_bk[2]; // ofs, len

} csa_t; // config status area

extern csa_t csa;
extern const csa_t csa_dft;
extern char cpu_id[25];
extern uint8_t bus_mac;
extern cd_spinlock_t p5_lock;


int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);
int flash_cal_crc(uint32_t src_addr, uint32_t len, uint16_t *crc);

extern list_head_t frame_free_head;

void common_service_init(void);
void common_service_routine(void);

void cd_main_early(void);
void cd_main_late(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

#endif
