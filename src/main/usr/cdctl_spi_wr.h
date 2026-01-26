/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDCTL_SPI_WR_H__
#define __CDCTL_SPI_WR_H__

#include "cd_utils.h"
#include "cd_list.h"

extern gpio_t cd_int_n;
extern gpio_t cd_cs;

#define CD_SS_HIGH()        gpio_set_high(&cd_cs)
#define CD_SS_LOW()         gpio_set_low(&cd_cs)
#define CD_INT_RD()         gpio_get_val(&cd_int_n)
#define CD_IRQ_ENABLE()     gpio_intr_enable(cd_int_n)
#define CD_IRQ_DISABLE()    gpio_intr_disable(cd_int_n)

extern uint8_t cdctl_buf[];

void cdctl_spi_wr_it(const uint8_t *w_buf, uint8_t *r_buf, int len);
void cdctl_spi_wr_init(void);

uint8_t cdctl_reg_r(uint8_t reg);
void cdctl_reg_w(uint8_t reg, uint8_t val);


static inline void cdctl_reg_r_it(uint8_t reg)
{
    cdctl_buf[0] = reg;
    CD_SS_LOW();
    cdctl_spi_wr_it(cdctl_buf, cdctl_buf, 2);
}

static inline void cdctl_reg_w_it(uint8_t reg, uint8_t val)
{
    cdctl_buf[0] = reg | 0x80;
    cdctl_buf[1] = val;
    CD_SS_LOW();
    cdctl_spi_wr_it(cdctl_buf, cdctl_buf, 2);
}

#endif
