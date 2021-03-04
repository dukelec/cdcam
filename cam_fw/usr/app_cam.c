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

int ov2640_init(void);

static cd_frame_t *frame_cam[2] = { NULL };
static cd_frame_t *frame_prepare = NULL;
static uint8_t frame_cnt = 0;
static uint8_t status = 0; // 0: idle, 1: first, 2: more
static bool err_flag = false;

static uint8_t pl_max = 220;
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
    }
}

void app_cam_init(void)
{
    ov2640_init();

    frame_prepare = list_get_entry(&frame_free_head, cd_frame_t);
    init_frame_cam(0);
    frame_prepare = list_get_entry(&frame_free_head, cd_frame_t);
    init_frame_cam(1);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void app_cam_routine(void)
{
    static int v_last = 0;
    int v_cur = GPIOB->IDR & 0x4;
    bool v_stop = v_last && !v_cur;
    v_last = v_cur;

    if (status && pl[pp_idx] >= pl_max) {
        err_flag = true;
        status = 0;
        d_error("cam: lost.\n");
    }

    if (status == 0) {
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

    update_prepare();
    if (frame_prepare && status && pl[!pp_idx] >= pl_max) {
        // 01: first, 10: more, 11: last
        frame_cam[!pp_idx]->dat[5] |= ((status == 1 ? 1 : 2) << 4) | (frame_cnt++ & 0xf);
        frame_cam[!pp_idx]->dat[2] += pl[!pp_idx];
        r_dev.cd_dev.put_tx_frame(&r_dev.cd_dev, frame_cam[!pp_idx]);
        init_frame_cam(!pp_idx);
        if (status == 1)
            status = 2;
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

    if (status == 0 && v_stop && csa.capture) {
        status = 1;
        csa.capture = false;
        d_debug("cam: cap...\n");
    }
}

void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
    if (status == 0 || pl[pp_idx] >= pl_max || (GPIOB->IDR & 0x6) != 0x6)
        return;
    *(pp[pp_idx] + pl[pp_idx]) = (uint8_t)GPIOA->IDR;
    if (++pl[pp_idx] >= pl_max)
        pp_idx = !pp_idx;
}

// vsync falling : start
// vsync rising : end

// or vsync rising: end / start
