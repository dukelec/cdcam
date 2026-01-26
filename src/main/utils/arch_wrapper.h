/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __ARCH_WRAPPER_H__
#define __ARCH_WRAPPER_H__

#include "hal/gpio_hal.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#define CSR_MSTATUS     0x300
#define SR_MIE          0x00000008
#define CSR_STATUS      CSR_MSTATUS
#define SR_IE           SR_MIE
#define __ASM_STR(x)    #x


#define csr_set(csr, val)                                \
({                                                       \
    unsigned long __v = (unsigned long)(val);            \
    __asm__ __volatile__ ("csrs " __ASM_STR(csr) ", %0"  \
                  : : "rK" (__v)                         \
                  : "memory");                           \
})

#define csr_read_clear(csr, val)                                \
({                                                              \
    unsigned long __v = (unsigned long)(val);                   \
    __asm__ __volatile__ ("csrrc %0, " __ASM_STR(csr) ", %1"    \
                  : "=r" (__v) : "rK" (__v)                     \
                  : "memory");                                  \
    __v;                                                        \
})

#define csr_clear(csr, val)                             \
({                                                      \
    unsigned long __v = (unsigned long)(val);           \
    __asm__ __volatile__ ("csrc " __ASM_STR(csr) ", %0" \
                  : : "rK" (__v)                        \
                  : "memory");                          \
})


#define local_irq_save(flags)       \
    do {                            \
        flags = _local_irq_save();  \
    } while (0)

static inline uint32_t _local_irq_save(void)
{
    return csr_read_clear(CSR_STATUS, SR_IE);
}

static inline void local_irq_restore(uint32_t flags)
{
    csr_set(CSR_STATUS, flags & SR_IE);
}

static inline void local_irq_enable(void)
{
    csr_set(CSR_STATUS, SR_IE);
}

static inline void local_irq_disable(void)
{
    csr_clear(CSR_STATUS, SR_IE);
}


// gpio wrapper

#define gpio_t  uint32_t


static inline bool gpio_get_val(gpio_t *gpio)
{
    return REG_GET_BIT(GPIO_IN_REG, BIT(*gpio));
}

static inline void gpio_set_val(gpio_t *gpio, bool value)
{
    if (value)
        REG_WRITE(GPIO_OUT_W1TS_REG, BIT(*gpio));
    else
        REG_WRITE(GPIO_OUT_W1TC_REG, BIT(*gpio));
}

static inline void gpio_set_high(gpio_t *gpio)
{
    REG_WRITE(GPIO_OUT_W1TS_REG, BIT(*gpio));
}

static inline void gpio_set_low(gpio_t *gpio)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT(*gpio));
}


static inline uint32_t get_systick(void)
{
    return esp_log_timestamp();
}

static inline void delay_ms(uint32_t val)
{
    esp_rom_delay_us(val * 1000);
}

#endif
