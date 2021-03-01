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

void app_cam_init(void)
{
    //cdn_sock_bind(&sock_raw_dbg);
    ov2640_init();

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void app_cam_routine(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 1000000 / SYSTICK_US_DIV) {
        t_l = get_systick();
    }
}

void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_0);
}
