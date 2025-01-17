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

extern spi_t r_spi;
int ov_out_size(uint16_t width,uint16_t height);
void ov_manual_exposure(uint16_t exposure);
void ov_manual_agc(uint8_t agc);
int ov2640_init(void);

static cd_frame_t *frame_cam[2] = { NULL };
static cd_frame_t *frame_prepare = NULL;
static uint8_t frame_cnt = 0;
static uint8_t status = 0; // 0: idle, 1: first, 2: more
static bool err_flag = false;

static uint8_t pl_max = 253 - 6;
static uint8_t pl[2] = { 0 }; // len
static uint8_t *pp[2] = { 0 };
static bool pp_idx = 0;


static inline void init_frame_cam(int idx)
{
    frame_cam[idx] = frame_prepare;
    frame_cam[idx]->dat[0] = csa.bus_cfg.mac;
    frame_cam[idx]->dat[1] = csa.cam_dst.addr[2];
    frame_cam[idx]->dat[2] = 3;
    frame_cam[idx]->dat[3] = 0x80;
    frame_cam[idx]->dat[4] = csa.cam_dst.port;
    // [5:4] FRAGMENT: 00: error, 01: first, 10: more, 11: last, [3:0]: cnt
    frame_cam[idx]->dat[5] = 0x40;

    pp[idx] = frame_cam[idx]->dat + 6;
    pl[idx] = 0;
    frame_prepare = NULL;
}

static inline void update_prepare(void)
{
    if (!frame_prepare)
        frame_prepare = list_get_entry(&frame_free_head, cd_frame_t);

    if (!frame_prepare && status) {
        status = 0;
        err_flag = true;
        d_error("cam: no free.\n");
    }
}

uint8_t cam_cfg_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    if (csa.manual) {
        d_info("cam_cfg: set exposure: %d, agc: 0x%02x\n", csa.exposure, csa.agc);
        ov_manual_exposure(csa.exposure);
        ov_manual_agc(csa.agc);
    }
    return 0;
}

void app_cam_init(void)
{
    ov2640_init();
    cdctl_reg_w(&r_dev, REG_INT_MASK, BIT_FLAG_TX_BUF_CLEAN);

    frame_prepare = list_get_entry(&frame_free_head, cd_frame_t);
    init_frame_cam(0);
    frame_prepare = list_get_entry(&frame_free_head, cd_frame_t);
    init_frame_cam(1);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
    __NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void app_cam_routine(void)
{
    static int v_last = 0;
    int v_cur = CAM_VSYNC_GPIO_Port->IDR & CAM_VSYNC_Pin;
    bool v_stop = v_last && !v_cur;
    v_last = v_cur;

    if (status == 0) {
        pl[0] = pl[1] = 0;
        frame_cnt = 0;

        if (v_stop && csa.capture) {
            d_debug("cam: cap...\n");
            if (csa.capture != 0xff)
                csa.capture = 0;
            status = 1;
        }
    }

    // speed up without using HAL library
    while (status) {
        //GPIOB->BSRR = 0x4000; // PB14

        update_prepare();
        if (!status)
            break;

        if (pl[0] >= pl_max && pl[1] >= pl_max) {
            err_flag = true;
            status = 0;
            d_error("cam: lost.\n");
        }

        if (pl[!pp_idx] >= pl_max) {
            cd_frame_t *frame = frame_cam[!pp_idx];
            frame_cam[!pp_idx]->dat[2] += pl[!pp_idx];
            init_frame_cam(!pp_idx);
            // 01: first, 10: more, 11: last
            frame->dat[5] |= ((status == 1 ? 1 : 2) << 4) | (frame_cnt++ & 0xf);
            list_put(&r_dev.tx_head, &frame->node);
            if (status == 1)
                status = 2;
        }

        if (!r_dev.is_pending && r_dev.tx_head.first) {
            cd_frame_t *frame = list_get_entry(&r_dev.tx_head, cd_frame_t);

            uint8_t *buf = frame->dat - 1;
            *buf = REG_TX | 0x80; // borrow space from the "node" item

            CD_CS_GPIO_Port->BRR = CD_CS_Pin; // cs = 0
            spi_wr(&r_spi, buf, NULL, buf[3] + 4);
            CD_CS_GPIO_Port->BSRR = CD_CS_Pin; // cs = 1

            r_dev.is_pending = true;
            list_put(&frame_free_head, &frame->node);
        }

        if (r_dev.is_pending && !(CD_INT_GPIO_Port->IDR & CD_INT_Pin)) {
            uint8_t buf[2] = {REG_TX_CTRL | 0x80, BIT_TX_START | BIT_TX_RST_POINTER};
            CD_CS_GPIO_Port->BRR = CD_CS_Pin; // cs = 0
            spi_wr(&r_spi, buf, NULL, 2);
            CD_CS_GPIO_Port->BSRR = CD_CS_Pin; // cs = 1
            r_dev.is_pending = false;
        }

        v_cur = CAM_VSYNC_GPIO_Port->IDR & CAM_VSYNC_Pin;
        v_stop = v_last && !v_cur;
        v_last = v_cur;
        if (v_stop)
            break;

        //GPIOB->BRR = 0x4000;
    }

    update_prepare();
    if (frame_prepare && status && v_stop) {
        // 01: first, 10: more, 11: last
        frame_cam[pp_idx]->dat[5] |= (3 << 4) | (frame_cnt & 0xf);
        frame_cam[pp_idx]->dat[2] += pl[pp_idx];
        r_dev.cd_dev.put_tx_frame(&r_dev.cd_dev, frame_cam[pp_idx]);
        init_frame_cam(pp_idx);
        d_debug("cam: done.\n");
        status = 0;
        pl[0] = pl[1] = 0;
        frame_cnt = 0;
    }

    update_prepare();
    if (frame_prepare && err_flag) {
        err_flag = false;
        frame_prepare->dat[0] = csa.bus_cfg.mac;
        frame_prepare->dat[1] = csa.cam_dst.addr[2];
        frame_prepare->dat[2] = 3;
        frame_prepare->dat[3] = 0x80;
        frame_prepare->dat[4] = csa.cam_dst.port;
        // [5:4] FRAGMENT: 00: error, 01: first, 10: more, 11: last, [3:0]: cnt
        frame_prepare->dat[5] = 0x40 | (frame_cnt & 0xf);
        r_dev.cd_dev.put_tx_frame(&r_dev.cd_dev, frame_prepare);
        frame_prepare = NULL;
    }
}

__attribute__((optimize("-Ofast"))) \
void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
    if (status == 0 || pl[pp_idx] >= pl_max || (GPIOB->IDR & 0x6) != 0x6) // CAM_HREF_Pin | CAM_VSYNC_Pin
        return;
    *(pp[pp_idx] + pl[pp_idx]) = GPIOA->IDR;
    if (++pl[pp_idx] >= pl_max)
        pp_idx = !pp_idx;

    //GPIOB->BSRR = 0x1000; // PB12
    //GPIOB->BRR = 0x1000;
}
