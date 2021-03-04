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

static list_head_t cam_pend = { 0 };
static cdn_sock_t sock_cam = { .port = 0x10, .ns = &dft_ns, .tx_only = true };
static cdn_pkt_t *pkt_cam[2] = { NULL };
static cdn_pkt_t *pkt_prepare = NULL;
static uint8_t pkt_cnt = 0;
static uint8_t status = 0; // 0: idle, 1: first, 2: more
static bool err_flag = false;

static uint8_t pl_max = 220;
static uint8_t pl[2] = { 0 }; // len
static uint8_t *pp[2] = { 0 };
static bool pp_idx = 0;


static void init_pkt_cam(int idx)
{
    pkt_cam[idx] = pkt_prepare;
    cdn_init_pkt(pkt_cam[idx]);
    pkt_cam[idx]->dst = csa.cam_dst;
    // [5:4] FRAGMENT: 00: error, 01: first, 10: more, 11: last, [3:0]: cnt
    pkt_cam[idx]->dat[0] = 0x40;
    pkt_cam[idx]->len = 1;

    pp[idx] = pkt_cam[idx]->dat + 1;
    pl[idx] = 0;
    pkt_prepare = NULL;
}

static void update_prepare(void)
{
    if (!pkt_prepare)
        pkt_prepare = cdn_pkt_get(&dft_ns.free_pkts);

    if (!pkt_prepare && status) {
        status = 0;
        err_flag = true;
    }
}

void app_cam_init(void)
{
    cdn_sock_bind(&sock_cam);
    ov2640_init();

    pkt_prepare = cdn_pkt_get(&dft_ns.free_pkts);
    init_pkt_cam(0);
    pkt_prepare = cdn_pkt_get(&dft_ns.free_pkts);
    init_pkt_cam(1);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);

    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void app_cam_routine(void)
{
    static int eee = 0;
    if (pl[pp_idx] >= pl_max && pl[!pp_idx] >= pl_max) {
        eee++;
    }

    static int t_l = 0;
    if (get_systick() - t_l > 2000000 / SYSTICK_US_DIV) {
        t_l = get_systick();
        //csa.capture = true;

        d_warn("eee...: %d\n", eee);
    }

    static int v_last = 0;
    int v_cur = GPIOB->IDR & 0x4;
    bool v_stop = v_last && !v_cur;
    v_last = v_cur;

    if (status == 0) {
        pl[0] = pl[1] = 0;
        pkt_cnt = 0;
    }

    update_prepare();
    if (pkt_prepare && err_flag) {
        err_flag = false;

        cdn_init_pkt(pkt_prepare);
        pkt_prepare->dst = csa.cam_dst;
        // [5:4] FRAGMENT: 00: error, 01: first, 10: more, 11: last, [3:0]: cnt
        pkt_prepare->dat[0] = 0x40 | (pkt_cnt & 0xf);
        pkt_prepare->len = 1;
        list_put(&cam_pend, &pkt_prepare->node);
        pkt_prepare = NULL;
    }

    update_prepare();
    if (pkt_prepare && status && pl[!pp_idx] >= pl_max) {
        // 01: first, 10: more, 11: last
        pkt_cam[!pp_idx]->dat[0] |= ((status == 1 ? 1 : 2) << 4) | (pkt_cnt++ & 0xf);
        pkt_cam[!pp_idx]->len += pl[!pp_idx];
        list_put(&cam_pend, &pkt_cam[!pp_idx]->node);
        init_pkt_cam(!pp_idx);
        if (status == 1)
            status = 2;
    }

    update_prepare();
    if (pkt_prepare && status && v_stop) {
        // 01: first, 10: more, 11: last
        pkt_cam[pp_idx]->dat[0] |= (3 << 4) | (pkt_cnt & 0xf);
        pkt_cam[pp_idx]->len += pl[pp_idx];
        list_put(&cam_pend, &pkt_cam[pp_idx]->node);
        init_pkt_cam(pp_idx);
        d_debug("cam: done.\n");
        status = 0;
        pl[0] = pl[1] = 0;
        pkt_cnt = 0;
    }

    if (status == 0 && v_stop && csa.capture) {
        status = 1;
        csa.capture = false;
        d_debug("cam: cap...\n");
    }

    if (frame_free_head.len > 1) {
        cdn_pkt_t *pkt = cdn_pkt_get(&cam_pend);
        if (pkt) {
            list_put(&dft_ns.free_pkts, &pkt->node);
            //cdn_sock_sendto(&sock_cam, pkt);
        }
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
