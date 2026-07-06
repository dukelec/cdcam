/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_main.h"
static const char *tag = "cd-main";

static gpio_t led_w_pin = LED_W_PIN;
static gpio_t led_g_pin = LED_G_PIN;

static gpio_t r_int = CDCTL_INT_PIN;
static gpio_t r_cs = CDCTL_CS_PIN;
static spi_t r_spi = {
        .ns_pin = &r_cs
};

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};
cdctl_dev_t r_dev = {0}; // CDBUS

TaskHandle_t dispatch_task_handle = NULL;


static void IRAM_ATTR gpio_isr_cd_int_n(void *arg)
{
    cdctl_int_isr(&r_dev);
}

static void IRAM_ATTR cdctl_spi_wr_isr(spi_transaction_t *t)
{
    if (t == &r_spi.trans)
        cdctl_spi_isr(&r_dev);
}


static void cdctl_spi_init(void)
{
    static spi_device_handle_t spi_dev = NULL;
    esp_err_t ret;

    gpio_set_val(&r_cs, 1);
    gpio_config_t io_r_cs = {
        .pin_bit_mask = (1ULL << r_cs),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_r_cs);

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
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev);
    ESP_ERROR_CHECK(ret);
    r_spi.dev = spi_dev;
}


// the 10MHz clock for cdctl is output by the bootloader (clk_out on mco pin)
void configure_led_pwm()
{
    ledc_timer_config_t ledc_timer_led = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_2,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 100000,
        .clk_cfg = LEDC_USE_XTAL_CLK
    };
    ledc_timer_config(&ledc_timer_led);

    ledc_channel_config_t led_channels[] = {
        { .gpio_num = led_w_pin, .channel = LEDC_CHANNEL_2 },
        { .gpio_num = led_g_pin, .channel = LEDC_CHANNEL_3 },
    };

    for (int i = 0; i < 2; i++) {
        led_channels[i].speed_mode = LEDC_LOW_SPEED_MODE;
        led_channels[i].timer_sel = LEDC_TIMER_2;
        led_channels[i].duty = 10; // 10/256 duty
        led_channels[i].hpoint = 0;
        ledc_channel_config(&led_channels[i]);
    }
}


static void led_set_w(uint8_t duty_w)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

static void led_set_g(uint8_t duty_g)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty_g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}


static void dispatch_task(void *arg)
{
    while (true) {
        if (!r_dev.rx_head.first) {
            ulTaskNotifyTake(pdTRUE, 100 / portTICK_PERIOD_MS);
            continue;
        }
        comm_service_poll();
        if (csa.capture_ctrl) {
            xTaskNotifyGive(rpt_task_handle);
            while (csa.capture_ctrl)
                ulTaskNotifyTake(pdTRUE, 10 / portTICK_PERIOD_MS);
        }
    }
}


static void led_task(void *arg)
{
    uint32_t t_last = get_systick();
    led_set_g(120);
    led_set_w(10);

    while (true) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if ((uint32_t)(get_systick() - t_last) >= 5000) {
            t_last = get_systick();
            ESP_LOGI(tag, "bus: %d, pend t %ld r %ld, irq %d, r %ld (lost %ld err %ld full %ld), t %ld (cd %ld err %ld)",
                    r_dev.state, r_dev.tx_head.len, r_dev.rx_head.len, !gpio_get_val(r_dev.int_n),
                    r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt, r_dev.rx_no_free_node_cnt,
                    r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);

            ESP_LOGI(tag, "free heap: %ld", esp_get_free_heap_size());
        }
    }
}


static int multi_output_vprintf(const char *fmt, va_list args) {
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        if (len >= (int)sizeof(buf))
            len = sizeof(buf) - 1; // vsnprintf returns would-be length on truncation
        if (len >= 2 && buf[len-1] == '\n' && buf[len-2] == '\n')
            len--; // "\n\n" -> "\n"
        fwrite(buf, 1, len, stdout);

        if (csa.dbg_en) {
            cd_frame_t *frm = cd_list_get(&frame_free_head);
            if (frm) {
                len = min(CDN_MAX_PAYLOAD, len);
                frm->dat[0] = bus_mac;
                frm->dat[1] = 0;
                frm->dat[2] = 2 + len;
                frm->dat[3] = 0x40;
                frm->dat[4] = 9;
                memcpy(frm->dat + 5, buf, len);
                cdctl_send_frame(&r_dev.cd_dev, frm);
            }
        }
    }
    return len;
}


void cd_main_early(void)
{
    ESP_LOGI(tag, "start cd_main_early ...\n");
    configure_led_pwm();

    for (int i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);

    load_conf();
    cdctl_spi_init();
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_int, CDCTL_INT_PIN);

    gpio_config_t io_conf_cd_int_n = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << r_int),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_cd_int_n);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(r_int, gpio_isr_cd_int_n, NULL);

    esp_log_set_vprintf(multi_output_vprintf);
}

void cd_main_late(void)
{
    ESP_LOGI(tag, "start cd_main_late ...\n");

    comm_service_init();
    xTaskCreate(dispatch_task, "dispatch_task", 4096, NULL, 20, &dispatch_task_handle);
    csa_list_show();
    xTaskCreate(led_task, "led_task", 4096, NULL, 1, NULL);
}


void cdctl_rx_cb(cdctl_dev_t *dev, cd_frame_t *frame)
{
    if (dispatch_task_handle) {
        BaseType_t task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(dispatch_task_handle, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
}
