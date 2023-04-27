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
uint8_t camctl_read_reg(camctl_dev_t *dev, uint8_t reg)
{
    uint8_t dat = 0xff;
    dev->manual_ctrl = true;
    while (dev->state > CAMCTL_IDLE);
    spi_mem_read(dev->spi, reg, &dat, 1);
    dev->manual_ctrl = false;
    if (!gpio_get_value(dev->int_n)) {
        uint32_t flags;
        local_irq_save(flags);
        camctl_int_isr(dev);
        local_irq_restore(flags);
    }
    return dat;
}

void camctl_write_reg(camctl_dev_t *dev, uint8_t reg, uint8_t val)
{
    dev->manual_ctrl = true;
    while (dev->state > CAMCTL_IDLE);
    spi_mem_write(dev->spi, reg | 0x80, &val, 1);
    dev->manual_ctrl = false;
    if (!gpio_get_value(dev->int_n)) {
        uint32_t flags;
        local_irq_save(flags);
        camctl_int_isr(dev);
        local_irq_restore(flags);
    }
}


void camctl_dev_init(camctl_dev_t *dev, list_head_t *free_head, spi_t *spi, gpio_t *int_n)
{
    if (!dev->name)
        dev->name = "camctl";
    dev->rx_frame = list_get_entry(free_head, cd_frame_t);
    dev->free_head = free_head;

#ifdef USE_DYNAMIC_INIT
    dev->state = CAMCTL_RST;
    dev->manual_ctrl = false;
    list_head_init(&dev->rx_head);
    dev->tx_cnt = 0;
    dev->rx_lost_cnt = 0;
    dev->rx_no_free_node_cnt = 0;
#endif

    dev->spi = spi;
    dev->int_n = int_n;

    dn_info(dev->name, "init...\n");

    uint8_t last_ver = 0xff;
    uint8_t same_cnt = 0;
    while (true) {
        uint8_t ver = camctl_read_reg(dev, CAM_REG_VERSION);
        if (ver != 0x00 && ver != 0xff && ver == last_ver) {
            if (same_cnt++ > 10)
                break;
        } else {
            last_ver = ver;
            same_cnt = 0;
        }
        debug_flush(false);
    }
    dn_info(dev->name, "version: %02x\n", last_ver);
    //camctl_write_reg(dev, CAM_REG_SETTING, setting);
    camctl_flush(dev);

    dn_debug(dev->name, "flags: %02x\n", camctl_read_reg(dev, CAM_REG_INT_FLAG));
    //camctl_write_reg(dev, CAM_REG_INT_MASK, CAMCTL_MASK);
    dev->state = CAMCTL_IDLE;
}


static inline
void camctl_read_reg_it(camctl_dev_t *dev, uint8_t reg)
{
    dev->buf[0] = reg;
    gpio_set_value(dev->spi->ns_pin, 0);
    spi_dma_write_read(dev->spi, dev->buf, dev->buf, 2);
}

/*
static inline
void camctl_write_reg_it(camctl_dev_t *dev, uint8_t reg, uint8_t val)
{
    dev->buf[0] = reg | 0x80;
    dev->buf[1] = val;
    gpio_set_value(dev->spi->ns_pin, 0);
    spi_dma_write(dev->spi, dev->buf, 2);
} */

// handlers

// int_n pin interrupt isr
void camctl_int_isr(camctl_dev_t *dev)
{
    if (!dev->manual_ctrl && dev->state == CAMCTL_IDLE) {
        dev->state = CAMCTL_RD_FLAG;
        camctl_read_reg_it(dev, CAM_REG_INT_FLAG);
    }
}

// dma finish callback
void camctl_spi_isr(camctl_dev_t *dev)
{
    // end of CAMCTL_RD_FLAG
    if (dev->state == CAMCTL_RD_FLAG) {
        uint8_t val = dev->buf[1];
        gpio_set_value(dev->spi->ns_pin, 1);

        // check rx error
        if (val & CAM_BIT_FLAG_RX_LOST)
            dev->rx_lost_cnt++;

        // check for new frame
        if (val & CAM_BIT_FLAG_RX_PENDING) {
            dev->state = CAMCTL_RX_PAGE_FLAG;
            dev->buf[0] = CAM_REG_RX_PAGE_FLAG;
            gpio_set_value(dev->spi->ns_pin, 0);
            spi_dma_write_read(dev->spi, dev->buf, dev->buf, 3);
            return;
        }

        dev->state = CAMCTL_IDLE;
        if (!gpio_get_value(dev->int_n))
            camctl_int_isr(dev);
        return;
    }

    // end of CAMCTL_RX_PAGE_FLAG
    if (dev->state == CAMCTL_RX_PAGE_FLAG) {
        gpio_set_value(dev->spi->ns_pin, 1);
        dev->rx_frame->dat[2] = 3 + dev->buf[1];
        dev->rx_frame->dat[5] = dev->buf[2];
        
        dev->state = CAMCTL_RX_HEADER;
        dev->buf[0] = CAM_REG_RX;
        gpio_set_value(dev->spi->ns_pin, 0);
        spi_dma_write(dev->spi, dev->buf, 1);
        return;
    }

    // end of CAMCTL_RX_HEADER
    if (dev->state == CAMCTL_RX_HEADER) {
        dev->state = CAMCTL_RX_BODY;
        if (dev->buf[1] != 0) {
            spi_dma_read(dev->spi, dev->rx_frame->dat + 6, dev->buf[1]);
            return; // should always return
        } // no return
    }
    
    // end of CAMCTL_RX_BODY
    if (dev->state == CAMCTL_RX_BODY) {
        gpio_set_value(dev->spi->ns_pin, 1);
        cd_frame_t *frame = list_get_entry_it(dev->free_head, cd_frame_t);
        if (frame) {
            list_put_it(&dev->rx_head, &dev->rx_frame->node);
            dev->rx_frame = frame;
            dev->rx_cnt++;
        } else {
            dev->rx_no_free_node_cnt++;
        }
        dev->state = CAMCTL_RD_FLAG;
        camctl_read_reg_it(dev, CAM_REG_INT_FLAG);
        return;
    }

    dn_warn(dev->name, "unexpected spi dma cb\n");
}
