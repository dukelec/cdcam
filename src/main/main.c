/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2025, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "main.h"
#include "cd_main.h"
#include <math.h>

static const char *TAG = "cdcam";
#define IMG_WIDTH       1600
#define IMG_HEIGHT      1600
#define IMG_YUV422_SIZE (IMG_WIDTH * IMG_HEIGHT * 2)

static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);

static cd_spinlock_t buf_lock = {0};
static uint8_t *yuv422_buf[3] = {0};

static uint8_t *raw_buf = NULL;
static uint8_t *jpg_buf = NULL;

static QueueHandle_t cam_notify_queue = NULL;
static esp_cam_sensor_device_t *cam_dev = NULL;


static inline void sent_cam_frame(cd_frame_t *frame)
{
    frame->dat[0] = csa.bus_cfg.mac;
    frame->dat[1] = csa.cam_dst.addr[2];
    // [5:4] FRAGMENT: 00: error, 01: first, 10: more, 11: last, [3:0]: cnt
    frame->dat[3] |= 0x40;
    frame->dat[4] = csa.cam_dst.port;
    cdctl_put_tx_frame(frame);
}


static uint8_t ov5647_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg)
{
    uint8_t read_buf = 0xff;
    esp_err_t ret = esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, &read_buf);
    if (ret)
        ESP_LOGE(TAG, "ov5647_read err: %d", ret);
    return read_buf;
}

static esp_err_t ov5647_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
}

static esp_err_t ov5647_ext_init(esp_sccb_io_handle_t sccb)
{
    ov5647_write(sccb, 0x350c, 0x00);
    ov5647_write(sccb, 0x350d, 0x00);
    ESP_LOGI(TAG, "ov5647: exposure 3500: %02x", ov5647_read(sccb, 0x3500));
    ESP_LOGI(TAG, "ov5647: exposure 3501: %02x", ov5647_read(sccb, 0x3501));
    ESP_LOGI(TAG, "ov5647: exposure 3502: %02x", ov5647_read(sccb, 0x3502));
    ESP_LOGI(TAG, "ov5647: manual ctrl: %02x", ov5647_read(sccb, 0x3503));
    ov5647_write(sccb, 0x3503, 0x07); // manual mode
    ov5647_write(sccb, 0x3500, 0x0f);
    //ov5647_write(sccb, 0x3501, 0xff);
    //ov5647_write(sccb, 0x3502, 0xff);
    ov5647_write(sccb, 0x350b, 0x10); // agc[7:0]
    ov5647_write(sccb, 0x350a, 0x00); // agc[9:8]
    ESP_LOGI(TAG, "ov5647: exposure 3500: %02x", ov5647_read(sccb, 0x3500));
    ESP_LOGI(TAG, "ov5647: exposure 3501: %02x", ov5647_read(sccb, 0x3501));
    ESP_LOGI(TAG, "ov5647: exposure 3502: %02x", ov5647_read(sccb, 0x3502));
    ESP_LOGI(TAG, "ov5647: pad_sel 3010: %02x", ov5647_read(sccb, 0x3010));
    ESP_LOGI(TAG, "ov5647: pad_out 300d: %02x", ov5647_read(sccb, 0x300d));
    ESP_LOGI(TAG, "ov5647: pad_oen 3002: %02x", ov5647_read(sccb, 0x3002));
    ov5647_write(sccb, 0x3010, 0x09);
    ov5647_write(sccb, 0x3002, 0x09);
    ESP_LOGI(TAG, "ov5647: pad_sel 3010: %02x", ov5647_read(sccb, 0x3010));
    ESP_LOGI(TAG, "ov5647: pad_oen 3002: %02x", ov5647_read(sccb, 0x3002));
    return 0;
}


static void common_task(void *arg)
{
    uint8_t led_val_bk = 0;
    while (true) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (led_val_bk != !!(csa.led_en & 2)) {
            ESP_LOGI(TAG, "led st: %d -> %d", led_val_bk, !led_val_bk);
            led_val_bk = !led_val_bk;
            ov5647_write(cam_dev->sccb_handle, 0x300d, led_val_bk ? 0x09 : 0x01);
        }
    }
}


static uint32_t s_gamma_curve(uint32_t x)
{
    return powf((float)x / 256, 0.7) * 256;
}


void app_main(void)
{
    esp_err_t ret = ESP_FAIL;
    cd_main_early();
    cd_main_late();

    jpeg_encoder_handle_t jpeg_handle;

    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 70,
    };

    jpeg_encode_memory_alloc_cfg_t rx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };

    jpeg_encode_memory_alloc_cfg_t tx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
    };

    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&encode_eng_cfg, &jpeg_handle));

    size_t tx_buffer_size = 0;
    raw_buf = (uint8_t*)jpeg_alloc_encoder_mem(IMG_WIDTH * IMG_HEIGHT, &tx_mem_cfg, &tx_buffer_size);
    if (raw_buf == NULL) {
        ESP_LOGE(TAG, "alloc raw buffer error");
        return;
    }

    size_t rx_buffer_size = 0;
    // assume that compression ratio of 10 to 1
    jpg_buf = (uint8_t*)jpeg_alloc_encoder_mem(IMG_WIDTH * IMG_HEIGHT / 2, &rx_mem_cfg, &rx_buffer_size);
    if (jpg_buf == NULL) {
        ESP_LOGE(TAG, "alloc jpg_buf error");
        return;
    }

    jpeg_encode_cfg_t enc_config = {
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = 80,
        .width = IMG_WIDTH / 2,
        .height = IMG_HEIGHT / 2,
    };


    // mipi ldo
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = 3,       // vo3
        .voltage_mv = 2500, // 2.5v
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    // camera Sensor and sccb init
    cdcam_sensor_init(I2C_NUM_0, &cam_dev);

    // csi init
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = IMG_WIDTH,
        .v_res = IMG_HEIGHT,
        .lane_bit_rate_mbps = CDCAM_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "csi init fail[%d]", ret);
        return;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "ops register fail");
        return;
    }

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

    // isp init
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = IMG_WIDTH,
        .v_res = IMG_HEIGHT,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    cam_notify_queue = xQueueCreate(2, sizeof(void *));
    for (int i = 0; i < 3; i++) {
        yuv422_buf[i] = aligned_alloc(0x40, IMG_YUV422_SIZE);
        if (!yuv422_buf[i]) {
            ESP_LOGE(TAG, "yuv422_buf[%d] alloc error", i);
            return;
        }
        ESP_LOGI(TAG, "yuv422_buf[%d] alloc: %p", i, yuv422_buf[i]);
    }

    if (esp_cam_ctlr_start(cam_handle) != ESP_OK) {
        ESP_LOGE(TAG, "driver start fail");
        return;
    }

    xTaskCreate(common_task, "common_task", 4096, NULL, 5, NULL);


#if 0
    esp_isp_bf_config_t bf_config = {
        .denoising_level = 5,
        .bf_template = {
            {1, 2, 1},
            {2, 4, 2},
            {1, 2, 1},
        },
    };
    ESP_ERROR_CHECK(esp_isp_bf_configure(isp_proc, &bf_config));
    ESP_ERROR_CHECK(esp_isp_bf_enable(isp_proc));
#endif

#if 0
    esp_isp_demosaic_config_t demosaic_config = {
        .grad_ratio = {
            .integer = 2,
            .decimal = 5,
        },
    };
    ESP_ERROR_CHECK(esp_isp_demosaic_configure(isp_proc, &demosaic_config));
    ESP_ERROR_CHECK(esp_isp_demosaic_enable(isp_proc));
#endif

#if 0
    esp_isp_sharpen_config_t sharpen_config = {
        .h_thresh = 255,
        .sharpen_template = {
            {1, 2, 1},
            {2, 4, 2},
            {1, 2, 1},
        },
    };
    ESP_ERROR_CHECK(esp_isp_sharpen_configure(isp_proc, &sharpen_config));
    ESP_ERROR_CHECK(esp_isp_sharpen_enable(isp_proc));
#endif

#if 1
    isp_gamma_curve_points_t pts = {};
    ESP_ERROR_CHECK(esp_isp_gamma_fill_curve_points(s_gamma_curve, &pts));
    ESP_ERROR_CHECK(esp_isp_gamma_configure(isp_proc, COLOR_COMPONENT_R, &pts));
    ESP_ERROR_CHECK(esp_isp_gamma_configure(isp_proc, COLOR_COMPONENT_G, &pts));
    ESP_ERROR_CHECK(esp_isp_gamma_configure(isp_proc, COLOR_COMPONENT_B, &pts));
    ESP_ERROR_CHECK(esp_isp_gamma_enable(isp_proc));
#endif

    ov5647_ext_init(cam_dev->sccb_handle);


    ppa_client_handle_t ppa_srm_handle = NULL;
    ppa_client_config_t ppa_srm_config = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_srm_config, &ppa_srm_handle));

    uint8_t *yuv422_buf_sm = aligned_alloc(0x40, IMG_YUV422_SIZE / 4);

    ppa_srm_oper_config_t srm_config = {
        .in.buffer = NULL,
        .in.pic_w = IMG_WIDTH,
        .in.pic_h = IMG_HEIGHT,
        .in.block_w = IMG_WIDTH,
        .in.block_h = IMG_HEIGHT,
        .in.block_offset_x = 0,
        .in.block_offset_y = 0,
        .in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer = yuv422_buf_sm,
        .out.buffer_size = IMG_YUV422_SIZE / 4,
        .out.pic_w = IMG_WIDTH / 2,
        .out.pic_h = IMG_HEIGHT / 2,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 0.5,
        .scale_y = 0.5,
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };


    while (true) {
        uint32_t flags;
        uint8_t *buf = NULL;
        xQueueReceive(cam_notify_queue, &buf, portMAX_DELAY);

        cd_irq_save(&buf_lock, flags);
        swap(yuv422_buf[1], yuv422_buf[2]);
        cd_irq_restore(&buf_lock, flags);
        buf = yuv422_buf[2];

        srm_config.in.buffer = buf;
        ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config));

        uint32_t jpg_size = 0;
        ESP_ERROR_CHECK(jpeg_encoder_process(jpeg_handle, &enc_config, yuv422_buf_sm, IMG_WIDTH * IMG_HEIGHT * 2 / 4,
                                             jpg_buf, IMG_WIDTH * IMG_HEIGHT / 2, &jpg_size));

        static int skip = 0;
        ESP_LOGI(TAG, "size : %d, skip: %d, cap: %d", jpg_size, skip, csa.capture);
        if (skip == 0 && csa.capture) {
            uint8_t cnt = 0;
            csa.img_len = jpg_size;
            uint8_t *pos = jpg_buf;
            unsigned len = csa.capture != 2 ? jpg_size : 0;
            while (csa.capture) {
                if (csa.capture == 2 && len == 0) {
                    if (csa.img_read_bk[1] == 0 && csa.img_read[1] != 0) {
                        cd_irq_save(&p5_lock, flags);
                        memcpy(csa.img_read_bk, csa.img_read, 8);
                        memset(csa.img_read, 0, 8);
                        cd_irq_restore(&p5_lock, flags);
                    }
                    if (csa.img_read_bk[0] >= jpg_size)
                        break;
                    pos = jpg_buf + csa.img_read_bk[0];
                    len = min(csa.img_read_bk[1], jpg_size - csa.img_read_bk[0]);
                    if (len == 0) {
                        vTaskDelay(0);
                        continue;
                    }
                    cd_irq_save(&p5_lock, flags);
                    memcpy(csa.img_read_bk, csa.img_read, 8);
                    memset(csa.img_read, 0, 8);
                    cd_irq_restore(&p5_lock, flags);
                }

                uint8_t l = min(251, len);
                cd_frame_t *frm = cd_list_get(&frame_free_head);
                if (!frm) {
                    ESP_LOGW(TAG, "no free frame");
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                    continue;
                }
                if (pos == jpg_buf)
                    frm->dat[3] = 0x10 | (cnt & 0xf);
                else if (pos + l < jpg_buf + jpg_size)
                    frm->dat[3] = 0x20 | (cnt & 0xf);
                else
                    frm->dat[3] = 0x30 | (cnt & 0xf);
                memcpy(frm->dat + 5, pos, l);
                frm->dat[2] = l + 2;
                sent_cam_frame(frm);
                cnt++;
                pos += l;
                len -= l;
                if (pos >= jpg_buf + jpg_size)
                    break;
            }
            csa.img_len = 0;
            memset(csa.img_read, 0, 16);
            if (csa.capture != 255)
                csa.capture = 0;
        }
        if (skip++ >= csa.skip || csa.capture == 0)
            skip = 0;
    }
}

static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    trans->buflen = IMG_YUV422_SIZE;
    trans->buffer = yuv422_buf[0];
    return false;
}

static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    uint32_t flags;
    BaseType_t task_woken = pdFALSE;
    cd_irq_save(&buf_lock, flags);
    swap(yuv422_buf[0], yuv422_buf[1]);
    cd_irq_restore(&buf_lock, flags);
    if (xQueueSendFromISR(cam_notify_queue, (void *) yuv422_buf[1], &task_woken) == pdPASS) {
        portYIELD_FROM_ISR(task_woken);
    }
    return false;
}
