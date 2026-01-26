/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_main.h"
#include "cdctl_spi_wr.h"

gpio_t cd_cs = CDCTL_CS_PIN;
gpio_t cd_int_n = CDCTL_INT_PIN;

static spi_device_handle_t cdspi = {0};
static spi_transaction_t cd_trans_it = {0};

uint8_t cdctl_buf[2];


static void cdctl_spi_wr(const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    esp_err_t ret;
    spi_transaction_t t = {
        .length = 8 * len,
        .tx_buffer = w_buf,
        .rx_buffer = r_buf
    };
    ret = spi_device_polling_transmit(cdspi, &t);
    assert(ret == ESP_OK);
}

void cdctl_spi_wr_it(const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    esp_err_t ret;
    memset(&cd_trans_it, 0, sizeof(cd_trans_it));
    cd_trans_it.length = 8 * len;
    cd_trans_it.tx_buffer = w_buf;
    cd_trans_it.rx_buffer = r_buf;
    ret = spi_device_queue_trans(cdspi, &cd_trans_it, portMAX_DELAY);
    assert(ret == ESP_OK);
}


static void IRAM_ATTR cdctl_spi_wr_isr(spi_transaction_t *t)
{
    if (t == &cd_trans_it)
        cdctl_spi_isr();
}


// used by init and user configuration, before enable isr
uint8_t cdctl_reg_r(uint8_t reg)
{
    volatile uint16_t dat = 0xffff;
    uint8_t tbuf[2] = {reg};
    CD_SS_LOW();
    cdctl_spi_wr(tbuf, (uint8_t *)&dat, 2);
    CD_SS_HIGH();
    return dat >> 8;
}

void cdctl_reg_w(uint8_t reg, uint8_t val)
{
    uint8_t tbuf[2] = {reg | 0x80, val};
    CD_SS_LOW();
    cdctl_spi_wr(tbuf, tbuf, 2);
    CD_SS_HIGH();
}


void cdctl_spi_wr_init(void)
{
    esp_err_t ret;

    gpio_set_val(&cd_cs, 1);
    gpio_config_t io_cd_cs = {
        .pin_bit_mask = (1ULL << cd_cs),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_cd_cs);

    spi_bus_config_t buscfg = {
        .miso_io_num = CDCTL_MISO_PIN,
        .mosi_io_num = CDCTL_MOSI_PIN,
        .sclk_io_num = CDCTL_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 257
    };

    spi_device_interface_config_t devcfg = {
        .clock_source = SPI_CLK_SRC_SPLL,
        .clock_speed_hz = 40000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .post_cb = cdctl_spi_wr_isr
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &cdspi);
    ESP_ERROR_CHECK(ret);
}
