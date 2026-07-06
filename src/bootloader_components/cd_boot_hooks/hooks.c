/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "hooks.h"
#include "modbus_crc.h"

#define APP_ADDR     0x20000
#define APP_MAX     0x100000
#define OTA_ADDR    0x120000


extern uint8_t fbuf[4096];

static spi_t cd_spi = {
        .cs = BL_CD_CS_PIN, .sck = BL_CD_SCK_PIN,
        .mosi = BL_CD_MOSI_PIN, .miso = BL_CD_MISO_PIN
};

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};
cdctl_dev_t r_dev = {0};    // CDBUS


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

static void ota_update(void) {
    ESP_LOGI("HOOK", "test read at 0x%08x buf: %08x", OTA_ADDR, (uint32_t)fbuf);
    int ret = esp_rom_spiflash_read(OTA_ADDR, (void *)fbuf, 256);
    if (!ret)
        hex_dump(fbuf, 256);
    
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

    ESP_LOGI("HOOK", "test read after erase at 0x%08x buf: %08x", OTA_ADDR, (uint32_t)fbuf);
    ret = esp_rom_spiflash_read(OTA_ADDR, (void *)fbuf, 256);
    if (!ret)
        hex_dump(fbuf, 256);

    ESP_LOGI("HOOK", "update succeed");
}


static void led_init(void)
{
    gpio_ll_set_level(&GPIO, BL_LED_PIN, 1);
    gpio_ll_func_sel(&GPIO, BL_LED_PIN, PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(BL_LED_PIN, SIG_GPIO_OUT_IDX, false, false);
    gpio_ll_output_enable(&GPIO, BL_LED_PIN);
}

static void led_set(bool on)
{
    gpio_ll_set_level(&GPIO, BL_LED_PIN, on);
}


static void wdt_feed(void)
{
    wdt_hal_context_t rwdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
    wdt_hal_write_protect_disable(&rwdt_ctx);
    wdt_hal_feed(&rwdt_ctx);
    wdt_hal_write_protect_enable(&rwdt_ctx);
}


void bootloader_after_init(void) {
    ESP_LOGI("HOOK", "cdcam bootloader, %s", BL_SW_VER);
    mco_clock_init();

    for (int i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);

    load_conf();
    init_info_str();
    ota_update();

    uint32_t bl_args = REG_READ(BL_ARGS_REG);
    REG_WRITE(BL_ARGS_REG, 0); // consume once; an unexpected reset falls back to normal
    if (bl_args == BL_ARGS_APP) {
        d_info("bl_comm: boot app (fast)\n");
        return;
    }
    csa.keep_in_bl = (bl_args == BL_ARGS_KEEP);

    led_init();
    esp_rom_delay_us(50000);
    cdctl_spi_init(&cd_spi);
    if (cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &cd_spi)) {
        d_error("bl_comm: no cdctl, skip\n");
        return;
    }

    // phase 1: talk at 115200 (a universally reachable rate), slow blink;
    // phase 2 (after BL_PHASE1_MS): switch to user baud, fast blink;
    // after BL_PHASE2_MS boot the app, unless a host asked to keep_in_bl.
    bool phase2 = csa.keep_in_bl;
    if (!phase2) {
        cdctl_set_baud_rate(&r_dev, 115200, 115200);
        cdctl_flush(&r_dev);
    }

    uint32_t t_boot = esp_log_early_timestamp();
    uint32_t t_blink = t_boot;
    bool led = true;
    led_set(led);

    while (true) {
        cdctl_poll(&r_dev);
        serial_cmd_dispatch();

        uint32_t now = esp_log_early_timestamp();
        if (now - t_blink > (phase2 ? BL_BLINK_FAST_MS : BL_BLINK_SLOW_MS)) {
            t_blink = now;
            led = !led;
            led_set(led);
        }

        if (csa.save_conf) {
            csa.save_conf = false;
            save_conf();
        }
        if (csa.do_reboot) {
            esp_rom_delay_us(10000);
            REG_WRITE(BL_ARGS_REG, 0xcdcd0000 | csa.do_reboot);
            d_info("bl_comm: reboot (%d)...\n", csa.do_reboot);
            esp_rom_software_reset_system();
        }

        if (!csa.keep_in_bl) {
            if (!phase2 && now - t_boot > BL_PHASE1_MS) {
                phase2 = true;
                cdctl_set_baud_rate(&r_dev, csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
                cdctl_flush(&r_dev);
            }
            if (now - t_boot > BL_PHASE2_MS)
                break; // boot app
        }
        wdt_feed();
    }
    led_set(0);
    d_info("bl_comm: exit\n");
}
