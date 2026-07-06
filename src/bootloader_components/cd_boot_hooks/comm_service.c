/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_utils.h"
#include "hooks.h"
#include "modbus_crc.h"

static char info_str[100];


void init_info_str(void)
{
    const char tlb[] = "0123456789abcdef";
    char cpu_id[13];
    // base mac is a 48bit big-endian value: (mac1 << 32) | mac0,
    // same byte order as esp_efuse_mac_get_default() used by the app
    uint32_t m0 = efuse_ll_get_mac0();
    uint32_t m1 = efuse_ll_get_mac1();
    uint8_t mac[6] = {
        (m1 >> 8) & 0xff, m1 & 0xff,
        (m0 >> 24) & 0xff, (m0 >> 16) & 0xff, (m0 >> 8) & 0xff, m0 & 0xff
    };
    for (int i = 0; i < 6; i++) {
        cpu_id[i * 2 + 0] = tlb[mac[i] >> 4];
        cpu_id[i * 2 + 1] = tlb[mac[i] & 0xf];
    }
    cpu_id[12] = '\0';
    strcpy(info_str, "M: cdcam (bl); S: ");
    strcat(info_str, cpu_id);
    strcat(info_str, "; SW: " BL_SW_VER);
    d_info("bl_comm: %s\n", info_str);
}


static void send_frame(cd_frame_t *frame, uint8_t p_len)
{
    frame->dat[1] = frame->dat[0];
    frame->dat[0] = csa.bus_cfg.mac;
    frame->dat[2] = p_len + 2;
    swap(frame->dat[3], frame->dat[4]); // swap src and dst port
    cdctl_send_frame(&r_dev.cd_dev, frame);
}

// device info
static void p1_handler(cd_frame_t *frame)
{
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;

    if (p_len == 0) {
        strcpy((char *)p_dat, info_str);
        send_frame(frame, strlen(info_str));
    } else {
        cd_list_put(&frame_free_head, frame);
    }
}

// csa manipulation
static void p5_handler(cd_frame_t *frame)
{
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;
    bool reply = !(*p_dat & 0x80);
    *p_dat &= 0x7f;

    if (*p_dat == 0x00 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_PAYLOAD - 1);
        memcpy(p_dat + 1, ((void *) &csa) + offset, len);
        *p_dat = 0;
        if (reply)
            send_frame(frame, len + 1);

    } else if (*p_dat == 0x20 && p_len > 3) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = p_len - 3;
        uint8_t *src_addr = p_dat + 3;
        uint16_t start = clip(offset, 0, sizeof(csa_t));
        uint16_t end = clip(offset + len, 0, sizeof(csa_t));
        memcpy(((void *) &csa) + start, src_addr + (start - offset), end - start);
        *p_dat = 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x01 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_PAYLOAD - 1);
        memcpy(p_dat + 1, ((void *) &csa_dft) + offset, len);
        *p_dat = 0;
        if (reply)
            send_frame(frame, len + 1);

    } else {
        cd_list_put(&frame_free_head, frame);
        return;
    }
    if (!reply)
        cd_list_put(&frame_free_head, frame);
}

// flash memory manipulation
static void p8_handler(cd_frame_t *frame)
{
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;
    bool reply = !(*p_dat & 0x80);
    *p_dat &= 0x7f;

    if (*p_dat == 0x2f && p_len == 9) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint32_t len = get_unaligned32(p_dat + 5);
        int ret = flash_erase(addr, len);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x00 && p_len == 6) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint8_t len = min(p_dat[5], CDN_MAX_PAYLOAD - 1);
        int ret = flash_read(addr, p_dat + 1, len);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, ret ? 1 : len + 1);

    } else if (*p_dat == 0x20 && p_len > 8) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint8_t len = p_len - 5;
        int ret = flash_write(addr, len, p_dat + 5);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x10 && p_len == 9) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint32_t len = get_unaligned32(p_dat + 5);
        uint16_t crc = 0xffff;
        int ret = flash_cal_crc(addr, len, &crc);
        if (ret == 0) {
            *p_dat = 0;
            put_unaligned16(crc, p_dat + 1);
        } else {
            *p_dat = 1;
        }
        if (reply)
            send_frame(frame, ret ? 1 : 3);

    } else {
        cd_list_put(&frame_free_head, frame);
        return;
    }
    if (!reply)
        cd_list_put(&frame_free_head, frame);
}


void serial_cmd_dispatch(void)
{
    cd_frame_t *frame = cdctl_recv_frame(&r_dev.cd_dev);
    if (!frame)
        return;

    uint8_t server_num = frame->dat[4];

    switch (server_num) {
    case 1: p1_handler(frame); break;
    case 5: p5_handler(frame); break;
    case 8: p8_handler(frame); break;
    default:
        d_error("bl_comm: cmd err ser_num: %d\n", server_num);
        cd_list_put(&frame_free_head, frame);
    }
}
