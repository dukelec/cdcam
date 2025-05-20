/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "camctl_it.h"
#include "cd_debug.h"

#define CAMCTL_MASK CAM_BIT_FLAG_RX_PENDING


// used by init and user configuration
uint8_t camctl_reg_r(camctl_dev_t *dev, uint8_t reg)
{
    uint8_t dat = 0xff;
    irq_disable(dev->int_irq);
    while (dev->state > CAMCTL_IDLE);
    spi_mem_read(dev->spi, reg, &dat, 1);
    irq_enable(dev->int_irq);
    return dat;
}

void camctl_reg_w(camctl_dev_t *dev, uint8_t reg, uint8_t val)
{
    irq_disable(dev->int_irq);
    while (dev->state > CAMCTL_IDLE);
    spi_mem_write(dev->spi, reg | 0x80, &val, 1);
    irq_enable(dev->int_irq);
}


void camctl_dev_init(camctl_dev_t *dev, list_head_t *free_head, spi_t *spi, gpio_t *int_n, irq_t int_irq)
{
    if (!dev->name)
        dev->name = "camctl";
    dev->rx_frame = cd_list_get(free_head);
    dev->free_head = free_head;

#ifdef USE_DYNAMIC_INIT
    dev->state = CAMCTL_RST;
    list_head_init(&dev->rx_head);
    dev->tx_cnt = 0;
    dev->rx_lost_cnt = 0;
    dev->rx_no_free_node_cnt = 0;
#endif

    dev->spi = spi;
    dev->int_n = int_n;
    dev->int_irq = int_irq;

    dn_info(dev->name, "init...\n");

    uint8_t last_ver = 0xff;
    uint8_t same_cnt = 0;
    while (true) {
        uint8_t ver = camctl_reg_r(dev, CAM_REG_VERSION);
        if (ver != 0x00 && ver != 0xff && ver == last_ver) {
            if (same_cnt++ > 10)
                break;
        } else {
            last_ver = ver;
            same_cnt = 0;
        }
    }
    dn_info(dev->name, "version: %02x\n", last_ver);
    //camctl_reg_w(dev, CAM_REG_SETTING, setting);
    camctl_flush(dev);

    dn_debug(dev->name, "flags: %02x\n", camctl_reg_r(dev, CAM_REG_INT_FLAG));
    dev->state = CAMCTL_IDLE;
    //camctl_reg_w(dev, CAM_REG_INT_MASK, CAMCTL_MASK);
}


static inline void camctl_reg_r_it(camctl_dev_t *dev, uint8_t reg)
{
    dev->buf[0] = reg;
    gpio_set_low(dev->spi->ns_pin);
    spi_wr_it(dev->spi, dev->buf, dev->buf, 2);
}

/*
static inline void camctl_reg_w_it(camctl_dev_t *dev, uint8_t reg, uint8_t val)
{
    dev->buf[0] = reg | 0x80;
    dev->buf[1] = val;
    gpio_set_low(dev->spi->ns_pin);
    spi_w_it(dev->spi, dev->buf, 2);
} */

// handlers

// int_n pin interrupt isr
void camctl_int_isr(camctl_dev_t *dev)
{
    if (dev->state == CAMCTL_IDLE) {
        dev->state = CAMCTL_RD_FLAG;
        camctl_reg_r_it(dev, CAM_REG_INT_FLAG);
    }
}

// dma finish callback
void camctl_spi_isr(camctl_dev_t *dev)
{
    // end of CAMCTL_RD_FLAG
    if (dev->state == CAMCTL_RD_FLAG) {
        uint8_t val = dev->buf[1];
        gpio_set_high(dev->spi->ns_pin);

        // check rx error
        if (val & CAM_BIT_FLAG_RX_LOST)
            dev->rx_lost_cnt++;

        // check for new frame
        if (val & CAM_BIT_FLAG_RX_PENDING) {
            dev->state = CAMCTL_RX_PAGE_FLAG;
            dev->buf[0] = CAM_REG_RX_PAGE_FLAG;
            gpio_set_low(dev->spi->ns_pin);
            spi_wr_it(dev->spi, dev->buf, dev->buf, 3);
            return;
        }

        dev->state = CAMCTL_IDLE;
        if (!gpio_get_val(dev->int_n))
            camctl_int_isr(dev);
        return;
    }

    // end of CAMCTL_RX_PAGE_FLAG
    if (dev->state == CAMCTL_RX_PAGE_FLAG) {
        gpio_set_high(dev->spi->ns_pin);
        dev->rx_frame->dat[2] = 2 + dev->buf[1];
        dev->rx_frame->dat[3] = dev->buf[2];
        
        dev->state = CAMCTL_RX_HEADER;
        dev->buf[0] = CAM_REG_RX;
        gpio_set_low(dev->spi->ns_pin);
        spi_wr_it(dev->spi, dev->buf, NULL, 1);
        return;
    }

    // end of CAMCTL_RX_HEADER
    if (dev->state == CAMCTL_RX_HEADER) {
        dev->state = CAMCTL_RX_BODY;
        if (dev->buf[1] != 0) {
            spi_wr_it(dev->spi, NULL, dev->rx_frame->dat + 5, dev->buf[1]);
            return; // should always return
        } // no return
    }
    
    // end of CAMCTL_RX_BODY
    if (dev->state == CAMCTL_RX_BODY) {
        gpio_set_high(dev->spi->ns_pin);
        cd_frame_t *frame = cd_list_get(dev->free_head);
        if (frame) {
            cd_list_put(&dev->rx_head, dev->rx_frame);
            dev->rx_frame = frame;
            dev->rx_cnt++;
        } else {
            dev->rx_no_free_node_cnt++;
        }
        dev->state = CAMCTL_RD_FLAG;
        camctl_reg_r_it(dev, CAM_REG_INT_FLAG);
        return;
    }

    dn_warn(dev->name, "unexpected spi dma cb\n");
}
