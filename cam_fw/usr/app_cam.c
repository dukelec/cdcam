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

extern DMA_HandleTypeDef hdma_dma_generator0;

int ov2640_init(void);

#define CIRC_BUF_SZ 1024
static uint8_t circ_buf[CIRC_BUF_SZ] = { 0x55, 0x56 };
static uint32_t rd_pos = 0;

static uint8_t buf_test[CIRC_BUF_SZ] = { 0xaa, 0xab };

void app_cam_init(void)
{
    //cdn_sock_bind(&sock_raw_dbg);
    ov2640_init();

    hdma_dma_generator0.XferAbortCallback = NULL;
    hdma_dma_generator0.XferCpltCallback = NULL;
    hdma_dma_generator0.XferErrorCallback = NULL;
    hdma_dma_generator0.XferHalfCpltCallback = NULL;
    //int ret = HAL_DMA_Start_IT(&hdma_dma_generator0, (uint32_t)&GPIOA->IDR, (uint32_t)circ_buf, CIRC_BUF_SZ);
    int ret = HAL_DMA_Start_IT(&hdma_dma_generator0, (uint32_t)buf_test, (uint32_t)circ_buf, CIRC_BUF_SZ);
    //int ret = HAL_DMA_Start_IT(&hdma_dma_generator0, (uint32_t)&GPIOA->IDR, (uint32_t)circ_buf, CIRC_BUF_SZ);

    ret |= HAL_DMAEx_EnableMuxRequestGenerator(&hdma_dma_generator0);
    d_info("start dma: %d, src: %x\n", ret, (uint32_t)&GPIOA->IDR);


    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void app_cam_routine(void)
{

    static int t_l = 0;
    if (get_systick() - t_l > 1000000 / SYSTICK_US_DIV) {
        t_l = get_systick();


        uint32_t wd_pos = CIRC_BUF_SZ - hdma_dma_generator0.Instance->CNDTR;
        d_info("wd_pos: %d, state: %d, dat0: %x, %x\n", wd_pos, hdma_dma_generator0.State, circ_buf[0], circ_buf[1]);

    }

}
