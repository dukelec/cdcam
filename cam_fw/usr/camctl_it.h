/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CAMCTL_IT_H__
#define __CAMCTL_IT_H__

#include "cdbus.h"

#define CAM_REG_VERSION         0x00
#define CAM_REG_PKT_SIZE        0x04
#define CAM_REG_INT_FLAG        0x10
#define CAM_REG_INT_MASK        0x11
#define CAM_REG_RX              0x14
#define CAM_REG_RX_CTRL         0x16
#define CAM_REG_RX_ADDR         0x18
#define CAM_REG_RX_PAGE_FLAG    0x19

#define CAM_BIT_FLAG_RX_PENDING         (1 << 1)
#define CAM_BIT_FLAG_RX_LOST            (1 << 3)

#define CAM_BIT_RX_RST_POINTER          (1 << 0)
#define CAM_BIT_RX_CLR_PENDING          (1 << 1)
#define CAM_BIT_RX_RST                  (1 << 4)
#define CAM_BIT_RX_RST_ALL              0x13


typedef enum {
    CAMCTL_RST = 0,

    CAMCTL_IDLE,
    CAMCTL_RD_FLAG,

    CAMCTL_RX_PAGE_FLAG,
    CAMCTL_RX_HEADER,
    CAMCTL_RX_BODY,
    CAMCTL_RX_CTRL
} camctl_state_t;

typedef struct {
    const char      *name;

    camctl_state_t  state;

    list_head_t     *free_head;
    list_head_t     rx_head;

    cd_frame_t      *rx_frame;

    uint8_t         buf[4];

    uint32_t        rx_cnt;
    uint32_t        rx_lost_cnt;
    uint32_t        rx_no_free_node_cnt;

    spi_t           *spi;
    gpio_t          *int_n;
    irq_t           int_irq;
} camctl_dev_t;


void camctl_dev_init(camctl_dev_t *dev, list_head_t *free_head, spi_t *spi, gpio_t *int_n, irq_t int_irq);

uint8_t camctl_reg_r(camctl_dev_t *dev, uint8_t reg);
void camctl_reg_w(camctl_dev_t *dev, uint8_t reg, uint8_t val);


static inline void camctl_flush(camctl_dev_t *dev)
{
    camctl_reg_w(dev, CAM_REG_RX_CTRL, CAM_BIT_RX_RST_ALL);
}

static inline cd_frame_t *camctl_get_rx_frame(camctl_dev_t *dev)
{
    return cd_list_get(&dev->rx_head);
}

void camctl_int_isr(camctl_dev_t *dev);
void camctl_spi_isr(camctl_dev_t *dev);

#endif
