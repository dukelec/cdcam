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

//static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_cs = { .group = CD_CS_GPIO_Port, .num = CD_CS_Pin };
spi_t r_spi = {
        .spi = SPI1,
        .ns_pin = &r_cs,
        .dma_rx = DMA1,
        .dma_ch_rx = DMA1_Channel1,
        .dma_ch_tx = DMA1_Channel2,
        .dma_mask = (2 << 0)
};

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

cdctl_dev_t r_dev = {0};    // CDBUS
cdn_ns_t dft_ns = {0};      // CDNET


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns, &packet_free_head, &frame_free_head);

    for (i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);
    for (i = 0; i < PACKET_MAX; i++)
        cdn_list_put(&packet_free_head, &packet_alloc[i]);

    spi_wr_init(&r_spi);
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, NULL);

    if (r_dev.version >= 0x10) {
        // 16MHz / (2 + 2) * (73 + 2) / 2^1 = 150MHz
        cdctl_reg_w(&r_dev, REG_PLL_N, 0x2);
        d_info("pll_n: %02x\n", cdctl_reg_r(&r_dev, REG_PLL_N));
        cdctl_reg_w(&r_dev, REG_PLL_ML, 0x49); // 0x49: 73
        d_info("pll_ml: %02x\n", cdctl_reg_r(&r_dev, REG_PLL_ML));

        d_info("pll_ctrl: %02x\n", cdctl_reg_r(&r_dev, REG_PLL_CTRL));
        cdctl_reg_w(&r_dev, REG_PLL_CTRL, 0x10); // enable pll
        d_info("clk_status: %02x\n", cdctl_reg_r(&r_dev, REG_CLK_STATUS));
        cdctl_reg_w(&r_dev, REG_CLK_CTRL, 0x01); // select pll

        d_info("clk_status after select pll: %02x\n", cdctl_reg_r(&r_dev, REG_CLK_STATUS));
        d_info("version after select pll: %02x\n", cdctl_reg_r(&r_dev, REG_VERSION));
    } else {
        d_info("fallback to cdctl-b1 module, ver: %02x\n", r_dev.version);
        CDCTL_SYS_CLK = 40000000; // 40MHz
        cdctl_set_baud_rate(&r_dev, csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
    }

    cdn_add_intf(&dft_ns, &r_dev.cd_dev, csa.bus_net, csa.bus_cfg.mac);
}


#if 0
static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        t_l = get_systick();

        d_debug("ctl: state %d, t_len %d, r_len %d, irq %d\n",
                r_dev.state, r_dev.tx_head.len, r_dev.rx_head.len,
                !gpio_get_val(r_dev.int_n));
        d_debug("  r_cnt %d (lost %d, err %d, no-free %d), t_cnt %d (cd %d, err %d)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt,
                r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
    }
}
#endif

void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    gpio_set_val(&led_r, 1);
    gpio_set_val(&led_g, 1);
    printf("\nstart app_main (cam)...\n");
    *stack_check = 0xababcdcd12123434;

    load_conf();
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    device_init();
    debug_flush(true);
    common_service_init();
    d_info("conf (cam): %s\n", csa.conf_from ? "load from flash" : "use default");
    csa_list_show();

    app_cam_init();
    gpio_set_val(&led_r, 0);

    while (true) {
        //dump_hw_status();
        app_cam_routine();
        cdctl_routine(&r_dev);
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        gpio_set_val(&led_cam, csa.led_en);
        debug_flush(false);

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}

