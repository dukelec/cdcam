/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "math.h"
#include "app_main.h"

int CDCTL_SYS_CLK = 150000000; // 150MHz for cdctl01a

gpio_t led_r = { .group = LED_R_GPIO_Port, .num = LED_R_Pin };
gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };
static gpio_t led_cam = { .group = LED_CAM_GPIO_Port, .num = LED_CAM_Pin };

uart_t debug_uart = { .huart = &huart1 };

static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_cs = { .group = CD_CS_GPIO_Port, .num = CD_CS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_cs };

gpio_t pga_rst = { .group = PGA_RST_GPIO_Port, .num = PGA_RST_Pin };
static gpio_t pga_int = { .group = PGA_INT_GPIO_Port, .num = PGA_INT_Pin };
gpio_t pga_cs = { .group = PGA_CS_GPIO_Port, .num = PGA_CS_Pin };
static spi_t pga_spi = { .hspi = &hspi2, .ns_pin = &pga_cs };

cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

cdctl_dev_t r_dev = {0};    // CDBUS
cdn_ns_t dft_ns = {0};      // CDNET

camctl_dev_t cam_dev = {0}; // Camera Controller


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns, &packet_free_head);

    for (i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);
    for (i = 0; i < PACKET_MAX; i++)
        list_put(&packet_free_head, &packet_alloc[i].node);

    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, NULL, &r_int);

    if (r_dev.version >= 0x10) {
        // 16MHz / (2 + 2) * (73 + 2) / 2^1 = 150MHz
        cdctl_write_reg(&r_dev, REG_PLL_N, 0x2);
        d_info("pll_n: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_N));
        cdctl_write_reg(&r_dev, REG_PLL_ML, 0x49); // 0x49: 73
        d_info("pll_ml: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_ML));

        d_info("pll_ctrl: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_CTRL));
        cdctl_write_reg(&r_dev, REG_PLL_CTRL, 0x10); // enable pll
        d_info("clk_status: %02x\n", cdctl_read_reg(&r_dev, REG_CLK_STATUS));
        cdctl_write_reg(&r_dev, REG_CLK_CTRL, 0x01); // select pll

        d_info("clk_status after select pll: %02x\n", cdctl_read_reg(&r_dev, REG_CLK_STATUS));
        d_info("version after select pll: %02x\n", cdctl_read_reg(&r_dev, REG_VERSION));
    } else {
        d_info("fallback to cdctl-b1 module, ver: %02x\n", r_dev.version);
        CDCTL_SYS_CLK = 40000000; // 40MHz
        cdctl_set_baud_rate(&r_dev, csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
    }

    cdn_add_intf(&dft_ns, &r_dev.cd_dev, csa.bus_net, csa.bus_cfg.mac);

    camctl_dev_init(&cam_dev, &frame_free_head, &pga_spi, &pga_int);
}


#if 1
static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        t_l = get_systick();

        d_debug("ctl: state %d, t_len %d, r_len %d, irq %d\n",
                r_dev.state, r_dev.tx_head.len, r_dev.rx_head.len,
                !gpio_get_value(r_dev.int_n));
        d_debug("  r_cnt %d (lost %d, err %d, no-free %d), t_cnt %d (cd %d, err %d)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt,
                r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
        d_debug("cam: state %d, r_len %d, irq %d | r_cnt %d (lost %d, no-free %d)\n",
                cam_dev.state, cam_dev.rx_head.len, !gpio_get_value(cam_dev.int_n),
                cam_dev.rx_cnt, cam_dev.rx_lost_cnt, cam_dev.rx_no_free_node_cnt);
    }
}
#endif

void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    gpio_set_value(&led_r, 1);
    gpio_set_value(&led_g, 1);
    printf("\nstart app_main (cam)...\n");
    *stack_check = 0xababcdcd12123434;

    load_conf();
    pga_config();
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    device_init();
    debug_flush(true);
    common_service_init();
    d_info("conf (cam): %s\n", csa.conf_from ? "load from flash" : "use default");
    csa_list_show();

    app_cam_init();
    gpio_set_value(&led_r, 0);

    while (true) {
        dump_hw_status();
        app_cam_routine();
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        gpio_set_value(&led_cam, csa.led_en);
        debug_flush(false);

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == r_int.num) {
        cdctl_int_isr(&r_dev);
    } else if (GPIO_Pin == pga_int.num) {
        camctl_int_isr(&cam_dev);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
        cdctl_spi_isr(&r_dev);
    else
        camctl_spi_isr(&cam_dev);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
        cdctl_spi_isr(&r_dev);
    else
        camctl_spi_isr(&cam_dev);
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
        cdctl_spi_isr(&r_dev);
    else
        camctl_spi_isr(&cam_dev);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    printf("spi error... [%08lx]\n", hspi->ErrorCode);
}
