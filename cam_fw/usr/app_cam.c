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

extern DMA_HandleTypeDef hdma_spi1_tx;
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

void app_cam_init(void)
{
    ov2640_init();
    cdctl_write_reg(&r_dev, REG_INT_MASK, BIT_FLAG_TX_BUF_CLEAN);

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
    int v_cur = GPIOB->IDR & 0x4;
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

    while (status) {
        //GPIOB->BSRR = 0x4000;

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

            GPIOA->BRR = 0x8000; // cs = 0
            *((volatile uint8_t *)&hspi1.Instance->DR) = REG_TX | 0x80;

            __HAL_DMA_DISABLE(&hdma_spi1_tx);
            hdma_spi1_tx.Instance->CNDTR = frame->dat[2] + 3;
            hdma_spi1_tx.Instance->CPAR = (uint32_t)&hspi1.Instance->DR;
            hdma_spi1_tx.Instance->CMAR = (uint32_t)frame->dat;
            __HAL_DMA_ENABLE(&hdma_spi1_tx);
            SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);

            while (hdma_spi1_tx.Instance->CNDTR != 0);
            while (hspi1.Instance->SR & SPI_FLAG_BSY);
            GPIOA->BSRR = 0x8000; // cs = 1

            r_dev.is_pending = true;
            list_put(&frame_free_head, &frame->node);
        }

        if (r_dev.is_pending && !(GPIOD->IDR & 8)) { // pd3
            GPIOA->BRR = 0x8000; // cs = 0
            *((volatile uint8_t *)&hspi1.Instance->DR) = REG_TX_CTRL | 0x80;
            while (!(hspi1.Instance->SR & SPI_FLAG_TXE));
            *((volatile uint8_t *)&hspi1.Instance->DR) = BIT_TX_START | BIT_TX_RST_POINTER;
            while (hspi1.Instance->SR & SPI_FLAG_BSY);
            GPIOA->BSRR = 0x8000; // cs = 1
            r_dev.is_pending = false;
        }

        v_cur = GPIOB->IDR & 0x4;
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

void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
    if (status == 0 || pl[pp_idx] >= pl_max || (GPIOB->IDR & 0x6) != 0x6)
        return;
    *(pp[pp_idx] + pl[pp_idx]) = GPIOA->IDR;
    if (++pl[pp_idx] >= pl_max)
        pp_idx = !pp_idx;

    //GPIOB->BSRR = 0x1000;
    //GPIOB->BRR = 0x1000;
}

// vsync falling : start
// vsync rising : end

// or vsync rising: end / start
