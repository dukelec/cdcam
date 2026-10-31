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


// move state to `to` if it is not above `from_max`, returns the state before
static camctl_state_t camctl_state_cas(camctl_dev_t *dev, camctl_state_t from_max, camctl_state_t to)
{
    uint32_t flags;
    cd_irq_save(&dev->lock, flags);
    camctl_state_t st = dev->state;
    if (st <= from_max)
        dev->state = to;
    cd_irq_restore(&dev->lock, flags);
    return st;
}

// take the spi from the state machine: wait for it to pause between two
// transfers, then park it in CAMCTL_REG_MANUAL so camctl_int_isr() stays out
static camctl_state_t camctl_manual_begin(camctl_dev_t *dev)
{
    camctl_state_t st; // spin with irq on, the dma irq has to finish the transfer
    while ((st = camctl_state_cas(dev, CAMCTL_IDLE, CAMCTL_REG_MANUAL)) > CAMCTL_IDLE);
    return st;
}

static void camctl_manual_end(camctl_dev_t *dev, camctl_state_t st)
{
    dev->state = st; // nobody else touches state while it is CAMCTL_REG_MANUAL
    if (st != CAMCTL_RST && !gpio_get_val(dev->int_n))
        camctl_int_isr(dev); // an int_n asserted meanwhile was turned away
}

// used by init and user configuration
uint8_t camctl_reg_r(camctl_dev_t *dev, uint8_t reg)
{
    uint8_t dat = 0xff;
    camctl_state_t st = camctl_manual_begin(dev);
    spi_mem_read(dev->spi, reg, &dat, 1);
    camctl_manual_end(dev, st);
    return dat;
}

void camctl_reg_w(camctl_dev_t *dev, uint8_t reg, uint8_t val)
{
    camctl_state_t st = camctl_manual_begin(dev);
    spi_mem_write(dev->spi, reg | 0x80, &val, 1);
    camctl_manual_end(dev, st);
}


void camctl_dev_init(camctl_dev_t *dev, list_head_t *free_head, spi_t *spi, gpio_t *int_n)
{
    if (!dev->name)
        dev->name = "camctl";
    dev->rx_frame = cd_list_get(free_head);
    dev->free_head = free_head;

#ifdef CD_USE_DYNAMIC_INIT
    dev->state = CAMCTL_RST;
    dev->lock = 0;
    list_head_init(&dev->rx_head);
    dev->rx_cnt = 0;
    dev->rx_lost_cnt = 0;
    dev->rx_no_free_node_cnt = 0;
#endif

    dev->spi = spi;
    dev->int_n = int_n;

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

// int_n pin interrupt isr: the idle check and the claim are one atomic step,
// so a caller from another context can not start a second spi transfer
void camctl_int_isr(camctl_dev_t *dev)
{
    if (dev->state == CAMCTL_RST) // before camctl_dev_init, RST only ever moves forward
        return;
    if (camctl_state_cas(dev, CAMCTL_IDLE, CAMCTL_RD_FLAG) <= CAMCTL_IDLE)
        camctl_reg_r_it(dev, CAM_REG_INT_FLAG);
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
