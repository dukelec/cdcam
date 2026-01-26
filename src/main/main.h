/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2025, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"
//#include "esp_event.h"
//#include "console/console.h"
#include "hal/cpu_hal.h"
#include "nvs_flash.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"


#include "esp_ldo_regulator.h"
#include "esp_cache.h"
#include "driver/jpeg_encode.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "driver/ppa.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"


#define LED_W_PIN       15
#define LED_G_PIN       14
#define MCO_PIN         6

#define CDCTL_MISO_PIN  10 // Q
#define CDCTL_MOSI_PIN  8  // D
#define CDCTL_SCK_PIN   9
#define CDCTL_CS_PIN    7
#define CDCTL_INT_PIN   11


#define CDCAM_RGB565_BITS_PER_PIXEL         16
#define CDCAM_MIPI_IDI_CLOCK_RATE           50000000
#define CDCAM_MIPI_CSI_LANE_BITRATE_MBPS    200 //line_rate = pclk * 4

#define CDCAM_CAM_SCCB_FREQ                 100000
#define CDCAM_CAM_SCCB_SCL_IO               26
#define CDCAM_CAM_SCCB_SDA_IO               27

#define CDCAM_CAM_FORMAT                    "MIPI_2lane_24Minput_RAW8_1600x1600"


extern TaskHandle_t dispatch_task_handle;

void cdcam_sensor_init(int i2c_port, esp_cam_sensor_device_t **out_cam_dev);


#endif
