/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_main.h"
static const char *tag = "comm-ser";

char cpu_id[25] = { 0 };
static char info_str[100];


static void send_frame(cd_frame_t *frame, uint8_t p_len)
{
    frame->dat[2] = p_len + 2;
    swap(frame->dat[0], frame->dat[1]); // swap mac
    swap(frame->dat[3], frame->dat[4]); // swap port
    cdctl_put_tx_frame(frame);
}

static void init_info_str(void)
{
    // M: model; S: serial string; HW: hardware version; SW: software version
    sprintf(info_str, "M: cdcam; S: %s; SW: %s", cpu_id, SW_VER);
    d_info("info: %s, git: %s\n", info_str, SW_VER_FULL);
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
        uint8_t ret = flash_erase(addr, len);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x00 && p_len == 6) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint8_t len = min(p_dat[5], CDN_MAX_DAT - 1);
        int ret = esp_flash_read(NULL, p_dat + 1, addr, len);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, ret ? 1 : len + 1);

    } else if (*p_dat == 0x20 && p_len > 8) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint8_t len = p_len - 5;
        uint8_t ret = flash_write(addr, len, p_dat + 5);
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

// csa manipulation
static void p5_handler(cd_frame_t *frame)
{
    static cd_spinlock_t p5_lock = {0};
    uint32_t flags;
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;
    bool reply = !(*p_dat & 0x80);
    *p_dat &= 0x7f;

    if (*p_dat == 0x00 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_DAT - 1);
        cd_irq_save(&p5_lock, flags);
        memcpy(p_dat + 1, ((void *) &csa) + offset, len);
        cd_irq_restore(&p5_lock, flags);
        *p_dat = 0;
        if (reply)
            send_frame(frame, len + 1);

    } else if (*p_dat == 0x20 && p_len > 3) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = p_len - 3;
        uint8_t *src_addr = p_dat + 3;
        uint16_t start = clip(offset, 0, sizeof(csa_t));
        uint16_t end = clip(offset + len, 0, sizeof(csa_t));
        cd_irq_save(&p5_lock, flags);
        memcpy(((void *) &csa) + start, src_addr + (start - offset), end - start);
        cd_irq_restore(&p5_lock, flags);
        *p_dat = 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x01 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_DAT - 1);
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


static inline void serial_cmd_dispatch(void)
{
    cd_frame_t *frame = cd_list_get(&cdctl_rx_head);

    if (frame) {
        uint8_t server_num = frame->dat[4];

        //ESP_LOGI(tag, "cmd %d\n", server_num);
        switch (server_num) {
        case 1: p1_handler(frame); break;
        case 5: p5_handler(frame); break;
        case 8: p8_handler(frame); break;
        default:
            ESP_LOGI(tag, "cmd err ser_num: %d\n", server_num);
            cd_list_put(&frame_free_head, frame);
        }
    }
}


void common_service_init(void)
{
    init_info_str();
}

void common_service_routine(void)
{
    serial_cmd_dispatch();

    if (csa.save_conf) {
        csa.save_conf = false;
        save_conf();
    }
    if (csa.do_reboot) {
        ESP_LOGI(tag, "do_reboot ...\n");
        esp_restart();
    }
}

