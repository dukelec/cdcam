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

// bit-bang spi backend for the bootloader stage
typedef struct {
    uint8_t     cs;
    uint8_t     sck;
    uint8_t     mosi;
    uint8_t     miso;
} spi_t;

// output the 10MHz clock for cdctl (xtal 40MHz / 4 via gpio-matrix clk_out)
void mco_clock_init(void);

// configure the bit-bang spi gpio pins
void cdctl_spi_init(spi_t *spi);

int spi_mem_read(spi_t *spi, uint8_t mem_addr, uint8_t *buf, int len);
int spi_mem_write(spi_t *spi, uint8_t mem_addr, const uint8_t *buf, int len);

#endif
