/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_main.h"
#include "esp_mac.h"
#include "esp_efuse.h"
#include "spi_flash_mmap.h"
static const char *tag = "comm-ser";

char cpu_id[25] = { 0 };
static char info_str[128];
cd_spinlock_t p5_lock = {0};


static void send_frame(cd_frame_t *frame, uint8_t p_len)
{
    frame->dat[2] = p_len + 2;
    frame->dat[1] = frame->dat[0];
    frame->dat[0] = bus_mac; // reply from own mac, even for broadcast cmd
    swap(frame->dat[3], frame->dat[4]); // swap port
    cdctl_send_frame(&r_dev.cd_dev, frame);
}

static void get_hw_ver_str(char *buf) // buf[24], efuse USER_DATA bytes: major, minor, patch; "" if never written
{
    uint8_t v[3] = { 0 };
    *buf = '\0';
    esp_efuse_read_block(EFUSE_BLK_USER_DATA, v, 0, 24);
    if (v[0] | v[1] | v[2])
        sprintf(buf, v[2] ? "; HW: v%d.%d.%d" : "; HW: v%d.%d", v[0], v[1], v[2]);
}

static void get_bl_ver_str(char *buf) // buf[32], "" for older bls
{
    const char *p;
    spi_flash_mmap_handle_t h;
    const int tag_len = sizeof(BL_VER_TAG) - 1;
    *buf = '\0';
    if (spi_flash_mmap(0, CONFIG_PARTITION_TABLE_OFFSET, SPI_FLASH_MMAP_DATA, (const void **)&p, &h) != ESP_OK)
        return;
    const char *end = p + CONFIG_PARTITION_TABLE_OFFSET;
    const char *s = memmem(p + CONFIG_BOOTLOADER_OFFSET_IN_FLASH, end - p - CONFIG_BOOTLOADER_OFFSET_IN_FLASH,
                           BL_VER_TAG, tag_len);
    if (s) {
        int n = end - s - tag_len;
        sprintf(buf, "; BL: %.*s", n < 20 ? n : 20, s + tag_len);
    }
    spi_flash_munmap(h);
}

static void init_info_str(void)
{
    uint8_t mac[6] = { 0 };
    esp_efuse_mac_get_default(mac);
    sprintf(cpu_id, "%02x%02x%02x%02x%02x%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // M: model; S: serial string; HW: hardware version; BL: bl version; SW: software version
    char hw_ver[24], bl_ver[32];
    get_hw_ver_str(hw_ver);
    get_bl_ver_str(bl_ver);
    snprintf(info_str, sizeof(info_str), "M: cdcam; S: %s%s%s; SW: %s", cpu_id, hw_ver, bl_ver, SW_VER);
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
        uint8_t len = min(p_dat[5], CDN_MAX_PAYLOAD - 1);
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

    } else if (*p_dat == 0x40 && p_len == 3) { // efuse USER_DATA read: 0x40, ofs_8, len_8 | [0x00, data]
        uint8_t ofs = p_dat[1], len = p_dat[2];
        int ret = ofs + len > 32 ? -1 : esp_efuse_read_block(EFUSE_BLK_USER_DATA, p_dat + 1, ofs * 8, len * 8);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, ret ? 1 : len + 1);

    } else if (*p_dat == 0x60 && p_len > 2) {
        // efuse USER_DATA write: 0x60, ofs_8 + [data] | [0x00]; rs coded, the block takes one write only
        uint8_t ofs = p_dat[1], len = p_len - 2;
        int ret = ofs + len > 32 ? -1 : esp_efuse_write_block(EFUSE_BLK_USER_DATA, p_dat + 2, ofs * 8, len * 8);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, 1);

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
    uint32_t flags;
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;
    bool reply = !(*p_dat & 0x80);
    *p_dat &= 0x7f;

    if (*p_dat == 0x00 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_PAYLOAD - 1);
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
        if (start == offsetof(csa_t, img_read) && csa.img_read[1]) {
            memcpy(csa.img_read_bk, csa.img_read, 8);
            memset(csa.img_read, 0, 8);
        }
        memcpy(((void *) &csa) + start, src_addr + (start - offset), end - start);
        cd_irq_restore(&p5_lock, flags);
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


static inline void serial_cmd_dispatch(void)
{
    cd_frame_t *frame = cdctl_recv_frame(&r_dev.cd_dev);

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


void comm_service_init(void)
{
    init_info_str();
}

void comm_service_poll(void)
{
    serial_cmd_dispatch();

    if (csa.save_conf) {
        csa.save_conf = false;
        save_conf();
    }
    if (csa.do_reboot) {
        // 1: reboot and stay in bootloader; 2: reboot and boot app
        REG_WRITE(BL_ARGS_REG, 0xcdcd0000 | csa.do_reboot);
        ESP_LOGI(tag, "do_reboot (%d) ...\n", csa.do_reboot);
        esp_restart();
    }
}

