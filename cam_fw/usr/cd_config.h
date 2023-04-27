/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CD_CONFIG_H__
#define __CD_CONFIG_H__

#define ARCH_SPI
#define ARCH_I2C
#define CD_LIST_IT

#define DEBUG
//#define VERBOSE
#define DBG_MIN_PKT         3
#define DBG_STR_LEN         160
#define DBG_TX_IT
//#define LIST_DEBUG

#define CDN_IRQ_SAFE

//#define CDN_SEQ
//#define CDN_RB_TREE
//#define CDN_L0_C
//#define CDN_L2

#include "main.h"  // generated by stm32cube
#include "debug_config.h"

#endif
