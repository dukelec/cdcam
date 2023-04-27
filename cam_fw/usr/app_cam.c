/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "math.h"
#include "app_main.h"
#include "lz4/lz4.h"
#include "pga_fw/pga_fw_00.h"
#include "pga_fw/pga_fw_01.h"
#include "pga_fw/pga_fw_02.h"

int ov_out_size(uint16_t width,uint16_t height);
int ov2640_init(void);

static uint8_t frame_cnt = 0;
static int status = 0; // 0: idle, 1: first, 2: more
static bool cam_enable = false;


void pga_config(void)
{
    uint8_t dummy_[13] = {0};
    uint8_t *fw_dat = (uint8_t *) frame_alloc;
    size_t fw_len = 0;
    int ret;
    printf("pb pga: start config...\n");

    gpio_set_value(&pga_cs, 0);
    gpio_set_value(&pga_rst, 0);
    delay_systick(4000 / SYSTICK_US_DIV);
    gpio_set_value(&pga_rst, 1);
    delay_systick(4000 / SYSTICK_US_DIV);
    gpio_set_value(&pga_cs, 1);
    HAL_SPI_Transmit(&hspi2, dummy_, 1, HAL_MAX_DELAY);
    gpio_set_value(&pga_cs, 0);

    fw_len = 24 * 1024;
    ret = lz4_decompress_unknownoutputsize(pga_fw_00_lz4, pga_fw_00_lz4_len, fw_dat, &fw_len);
    printf("ret: %d, out_size: %u\n", ret, fw_len);
    HAL_SPI_Transmit(&hspi2, fw_dat, fw_len, HAL_MAX_DELAY);

    fw_len = 24 * 1024;
    ret = lz4_decompress_unknownoutputsize(pga_fw_01_lz4, pga_fw_01_lz4_len, fw_dat, &fw_len);
    printf("ret: %d, out_size: %u\n", ret, fw_len);
    HAL_SPI_Transmit(&hspi2, fw_dat, fw_len, HAL_MAX_DELAY);

    fw_len = 24 * 1024;
    ret = lz4_decompress_unknownoutputsize(pga_fw_02_lz4, pga_fw_02_lz4_len, fw_dat, &fw_len);
    printf("ret: %d, out_size: %u\n", ret, fw_len);
    HAL_SPI_Transmit(&hspi2, fw_dat, fw_len, HAL_MAX_DELAY);

    gpio_set_value(&pga_cs, 1);
    HAL_SPI_Transmit(&hspi2, dummy_, 13, HAL_MAX_DELAY);
    printf("pb pga: done\n");
}


static inline void sent_cam_frame(cd_frame_t *frame)
{
    frame->dat[0] = csa.bus_cfg.mac;
    frame->dat[1] = csa.cam_dst.addr[2];
    frame->dat[3] = 0x80;
    frame->dat[4] = csa.cam_dst.port;
    // [5:4] FRAGMENT: 00: error, 01: first, 10: more, 11: last, [3:0]: cnt
    frame->dat[5] |= 0x40;
    r_dev.cd_dev.put_tx_frame(&r_dev.cd_dev, frame);
}

uint8_t cam_cfg_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    d_info("cam_cfg: set size: %dx%d\n", csa.width, csa.height);
    ov_out_size(csa.width, csa.height);
    return 0;
}

void app_cam_init(void)
{
    ov2640_init();
    printf("pga pkt_size: %d + 1\n", camctl_read_reg(&cam_dev, CAM_REG_PKT_SIZE));
}

void app_cam_routine(void)
{
    if (status == 0) {
        if (csa.capture) {
            //d_debug("cam: cap...\n");
            if (csa.capture != 0xff) {
                csa.capture = 0;
            }
            status = 1;
            if (!cam_enable) {
                cam_enable = true;
                camctl_flush(&cam_dev);
                camctl_read_reg(&cam_dev, CAM_REG_INT_FLAG); // clear lost flag
                camctl_write_reg(&cam_dev, CAM_REG_INT_MASK, CAM_BIT_FLAG_RX_PENDING);
            }
        } else {
            cd_frame_t *frame = camctl_get_rx_frame(&cam_dev);
            if (frame) {
                list_put(&frame_free_head, &frame->node);
                //d_debug("cam: drop\n");
            }
            if (cam_enable) {
                cam_enable = false;
                camctl_write_reg(&cam_dev, CAM_REG_INT_MASK, 0);
            }
        }
    } else {
        cd_frame_t *frame = camctl_get_rx_frame(&cam_dev);
        if (frame) {
            //printf("f size: %d, flag: %02x\n", frame->dat[2], frame->dat[5]);
            if (status == 1) {
                if (frame->dat[5] != 0x10) {
                    list_put(&frame_free_head, &frame->node);
                    //d_debug("cam: drop & wait\n");
                } else {
                    status = 2;
                    frame_cnt = 1;
                    sent_cam_frame(frame);
                }
            } else {
                uint8_t type = frame->dat[5] >> 4;
                uint8_t cnt = frame->dat[5] & 0xf;
                if ((type == 2 || type == 3) && cnt == frame_cnt) {
                    sent_cam_frame(frame);
                } else {
                    d_debug("cam: err, flg: %02x, cnt: %02x\n", frame->dat[5], frame_cnt);
                    frame->dat[5] = cnt; // change type to err
                    sent_cam_frame(frame);
                }
                if (type == 3) {
                    status = 0;
                    //d_debug("cam: done\n");
                }
                frame_cnt++;
                frame_cnt &= 0x0f;
            }
        }
    }
}
