/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_utils.h"
#include "hooks.h"


static uint8_t spi_xfer(spi_t *spi, uint8_t tx)
{
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--) {
        gpio_ll_set_level(&GPIO, spi->mosi, (tx >> i) & 1);
        gpio_ll_set_level(&GPIO, spi->sck, 1);
        rx = (rx << 1) | gpio_ll_get_level(&GPIO, spi->miso);
        gpio_ll_set_level(&GPIO, spi->sck, 0);
    }
    return rx;
}

int spi_mem_read(spi_t *spi, uint8_t mem_addr, uint8_t *buf, int len)
{
    gpio_ll_set_level(&GPIO, spi->cs, 0);
    spi_xfer(spi, mem_addr);
    for (int i = 0; i < len; i++)
        buf[i] = spi_xfer(spi, 0);
    gpio_ll_set_level(&GPIO, spi->cs, 1);
    return 0;
}

int spi_mem_write(spi_t *spi, uint8_t mem_addr, const uint8_t *buf, int len)
{
    gpio_ll_set_level(&GPIO, spi->cs, 0);
    spi_xfer(spi, mem_addr);
    for (int i = 0; i < len; i++)
        spi_xfer(spi, buf[i]);
    gpio_ll_set_level(&GPIO, spi->cs, 1);
    return 0;
}


static void gpio_out_init(uint32_t pin, bool val)
{
    gpio_ll_set_level(&GPIO, pin, val);
    gpio_ll_func_sel(&GPIO, pin, PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    gpio_ll_output_enable(&GPIO, pin);
}

void mco_clock_init(void)
{
    // 10MHz clock for cdctl: xtal 40MHz / 4 -> clk_out dbg ch0 -> mco pin
    clk_ll_bind_output_channel(CLKOUT_SIG_XTAL, CLKOUT_CHANNEL_1);
    clk_ll_set_output_channel_divider(CLKOUT_CHANNEL_1, 4);
    clk_ll_enable_output_channel(CLKOUT_CHANNEL_1, true);
    gpio_ll_func_sel(&GPIO, BL_MCO_PIN, PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(BL_MCO_PIN, DBG_CH0_CLK_IDX, false, false);
    gpio_ll_output_enable(&GPIO, BL_MCO_PIN);
    esp_rom_delay_us(1000); // wait clock stable
}

void cdctl_spi_init(spi_t *spi)
{
    gpio_out_init(spi->cs, 1);
    gpio_out_init(spi->sck, 0);
    gpio_out_init(spi->mosi, 0);

    gpio_ll_func_sel(&GPIO, spi->miso, PIN_FUNC_GPIO);
    gpio_ll_pullup_en(&GPIO, spi->miso);
    gpio_ll_input_enable(&GPIO, spi->miso);
}
