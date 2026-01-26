/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "esp_log.h"
#include "esp_rom_spiflash.h"
#include "modbus_crc.h"

#define min(a, b) ((a) < (b) ? (a) : (b))

#define APP_ADDR     0x20000
#define APP_MAX     0x100000
#define OTA_ADDR    0x120000


static uint8_t buffer[4096] __attribute__((aligned(4)));


static void hex_dump(const void *addr, int len)
{
    int i;
    char sbuf[17];
    const uint8_t *pc = (const uint8_t *)addr;

    if (len == 0 || len < 0)
        return;

    for (i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            if (i != 0) {
                esp_rom_printf("  %s\n", sbuf);
            }
            esp_rom_printf("  %04x ", i); // offset
        }
        esp_rom_printf(" %02x", pc[i]);

        // printable ascii character
        if (pc[i] < 0x20 || pc[i] > 0x7e)
            sbuf[i % 16] = '.';
        else
            sbuf[i % 16] = pc[i];
        sbuf[(i % 16) + 1] = '\0';
    }

    // pad out last line
    while ((i % 16) != 0) {
        esp_rom_printf("   ");
        i++;
    }
    // print the final ascii field
    esp_rom_printf("  %s\n", sbuf);
}


static int flash_cal_crc(uint32_t src_addr, uint32_t len, uint16_t *crc)
{
    uint16_t c = 0xffff;
    uint32_t p = src_addr;
    while (true) {
        uint32_t left_len = len - (p - src_addr);
        uint32_t sub_len = min(left_len, 4096);
        if (sub_len == 0)
            break;
        uint32_t copy_len = (sub_len + 3) & ~3;
        int ret = esp_rom_spiflash_read(p, (void *)buffer, copy_len);
        if (ret != 0)
            return ret;
        c = crc16_sub(buffer, sub_len, c);
        p += sub_len;
    }
    *crc = c;
    return 0;
}

static int flash_move(uint32_t src_addr, uint32_t dst_addr, uint32_t len)
{
    uint32_t p = src_addr;
    while (true) {
        uint32_t left_len = len - (p - src_addr);
        uint32_t sub_len = min(left_len, 4096);
        if (sub_len == 0)
            break;
        uint32_t copy_len = (sub_len + 3) & ~3;
        int ret = esp_rom_spiflash_read(p, (void *)buffer, copy_len);
        if (ret != 0)
            return ret;
        ret = esp_rom_spiflash_write(dst_addr + (p - src_addr), (void *)buffer, copy_len);
        if (ret != 0)
            return ret;
        p += sub_len;
    }
    return 0;
}


/* Function used to tell the linker to include this file
 * with all its symbols.
 */
void bootloader_hooks_include(void){
}


void bootloader_before_init(void) {
    /* Keep in my mind that a lot of functions cannot be called from here
     * as system initialization has not been performed yet, including
     * BSS, SPI flash, or memory protection. */
    //ESP_LOGI("HOOK", "This hook is called BEFORE bootloader initialization");
}

void bootloader_after_init(void) {
    ESP_LOGI("HOOK", "cdcam bootloader, v1.2");
    ESP_LOGI("HOOK", "test read at 0x%08x buf: %08x", OTA_ADDR, (uint32_t)buffer);
    int ret = esp_rom_spiflash_read(OTA_ADDR, (void *)buffer, 256);
    if (!ret)
        hex_dump(buffer, 256);
    
    uint32_t hdr;
    ret = esp_rom_spiflash_read(OTA_ADDR, &hdr, 4);

    if (ret != 0) {
        ESP_LOGE("HOOK", "read hdr err (%d)", ret);
        return;
    }

    if ((hdr >> 24) != 0xcd) {
        ESP_LOGI("HOOK", "no fw found (%08x)", hdr);
        return;
    }

    hdr &= 0x00ffffff;
    if (!hdr || hdr > APP_MAX) {
        ESP_LOGE("HOOK", "hdr len err");
        return;
    }

    uint16_t crc = -1;
    ret = flash_cal_crc(OTA_ADDR + 4, hdr + 2, &crc);
    if (ret != 0 || crc != 0) {
        ESP_LOGE("HOOK", "fw file crc err %d (%04x)", ret, crc);
        return;
    }

    int sectors = (hdr + 4095) / 4096;
    ESP_LOGI("HOOK", "erase fw at 0x%08x, len: %d, sectors: %d", APP_ADDR, hdr, sectors);
    for (int i = 0; i < sectors; i++) {
        ret = esp_rom_spiflash_erase_sector((APP_ADDR + 4096 * i) / 4096);
        if (ret != 0) {
            ESP_LOGE("HOOK", "erase fw err %d", ret);
            esp_rom_software_reset_cpu(0);
            return;
        }
    }

    ESP_LOGI("HOOK", "copy fw ...");
    ret = flash_move(OTA_ADDR + 4, APP_ADDR, hdr);
    if (ret != 0) {
        ESP_LOGE("HOOK", "copy fw err %d", ret);
        esp_rom_software_reset_cpu(0);
        return;
    }

    ESP_LOGI("HOOK", "verify fw ...");
    uint16_t crc_src = -1;
    uint16_t crc_dst = -1;
    ret = flash_cal_crc(OTA_ADDR + 4, hdr, &crc_src);
    ret |= flash_cal_crc(APP_ADDR, hdr, &crc_dst);
    if (ret != 0 || crc_dst != crc_src) {
        ESP_LOGE("HOOK", "new fw crc err %d (dst %04x != src %04x)", ret, crc_dst, crc_src);
        esp_rom_software_reset_cpu(0);
        return;
    }

    ESP_LOGI("HOOK", "remove fw file ...");
    ret = esp_rom_spiflash_erase_sector(OTA_ADDR / 4096);
    if (ret != 0) {
        ESP_LOGE("HOOK", "erase fw file err %d", ret);
        return;
    }

    ESP_LOGI("HOOK", "test read after erase at 0x%08x buf: %08x", OTA_ADDR, (uint32_t)buffer);
    ret = esp_rom_spiflash_read(OTA_ADDR, (void *)buffer, 256);
    if (!ret)
        hex_dump(buffer, 256);

    ESP_LOGI("HOOK", "update succeed");
}
