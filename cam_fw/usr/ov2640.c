#include "app_main.h"
#include "ov2640.h"

extern I2C_HandleTypeDef hi2c2;

static i2c_t ov_dev = { .hi2c = &hi2c2, .dev_addr = 0x60 };
static gpio_t ov_rst = { .group = CAM_RST_GPIO_Port, .num = CAM_RST_Pin };

static uint8_t ov_read_reg(i2c_t *dev, uint8_t reg)
{
    uint8_t dat = 0xff;
    i2c_mem_read(dev, reg, &dat, 1);
    return dat;
}

static void ov_write_reg(i2c_t *dev, uint8_t reg, uint8_t val)
{
    i2c_mem_write(dev, reg, &val, 1);
}


void ov_jpeg_mode(void)
{
    for (int i = 0; i < sizeof(ov_yuv422_tbl) / 2; i++)
        ov_write_reg(&ov_dev, ov_yuv422_tbl[i][0], ov_yuv422_tbl[i][1]);

    for (int i = 0; i < sizeof(ov_jpeg_tbl) / 2; i++)
        ov_write_reg(&ov_dev, ov_jpeg_tbl[i][0], ov_jpeg_tbl[i][1]);
}

// level: 0~4
void ov_auto_exposure(uint8_t level)
{  
    const uint8_t *p = ov_auto_exposure_tbl[level];
    for (int i = 0; i < 4; i++)
        ov_write_reg(&ov_dev, p[i*2], p[i*2+1]);
}

void ov_manual_exposure(uint16_t exposure)
{
    ov_write_reg(&ov_dev, 0xff, 0x01);
    ov_write_reg(&ov_dev, 0x13, ov_read_reg(&ov_dev, 0x13) & 0xfe); // clear bit0
    ov_write_reg(&ov_dev, 0x45, (ov_read_reg(&ov_dev, 0x45) & 0xc0) | (exposure >> 10));
    ov_write_reg(&ov_dev, 0x10, (exposure >> 2) & 0xff);
    ov_write_reg(&ov_dev, 0x04, (ov_read_reg(&ov_dev, 0x04) & 0xfc) | (exposure & 0x3));
}

void ov_manual_agc(uint8_t agc)
{
    ov_write_reg(&ov_dev, 0xff, 0x01);
    ov_write_reg(&ov_dev, 0x13, ov_read_reg(&ov_dev, 0x13) & 0xfb); // clear bit2
    ov_write_reg(&ov_dev, 0x14, ov_read_reg(&ov_dev, 0x14) & 0x1f); // clear bit[7:5]: agc gain ceiling, GH[2:0]
    ov_write_reg(&ov_dev, 0x00, agc);
    ov_write_reg(&ov_dev, 0x45, ov_read_reg(&ov_dev, 0x45) & 0x3f); // clear bit[7:6]: agc[9:8]
}

void ov_light_mode(uint8_t mode)
{
    uint8_t reg_cc, reg_cd, reg_ce;

    switch (mode) {
    case 1: // sunny
        reg_cc = 0x5E;
        reg_cd = 0x41;
        reg_ce = 0x54;
        break;
    case 2: // cloudy
        reg_cc = 0x65;
        reg_cd = 0x41;
        reg_ce = 0x4F;
        break;
    case 3: // office
        reg_cc = 0x52;
        reg_cd = 0x41;
        reg_ce = 0x66;
        break;
    case 4: // home
        reg_cc = 0x42;
        reg_cd = 0x3F;
        reg_ce = 0x71;
        break;
    default: // auto
        ov_write_reg(&ov_dev, 0xFF, 0x00);
        ov_write_reg(&ov_dev, 0xC7, 0x10); // AWB ON
        return;
    }
    ov_write_reg(&ov_dev, 0xFF, 0x00);
    ov_write_reg(&ov_dev, 0xC7, 0x40); // AWB OFF
    ov_write_reg(&ov_dev, 0xCC, reg_cc);
    ov_write_reg(&ov_dev, 0xCD, reg_cd);
    ov_write_reg(&ov_dev, 0xCE, reg_ce);
}

// range: [-2, +2]
void ov_color_saturation(int8_t sat)
{
    uint8_t reg_7d = ((sat + 4) << 4) | 0x08;
    ov_write_reg(&ov_dev, 0xFF, 0x00);
    ov_write_reg(&ov_dev, 0x7C, 0x00);
    ov_write_reg(&ov_dev, 0x7D, 0x02);
    ov_write_reg(&ov_dev, 0x7C, 0x03);
    ov_write_reg(&ov_dev, 0x7D, reg_7d);
    ov_write_reg(&ov_dev, 0x7D, reg_7d);
}

// range: [-2, +2]
void ov_brightness(int8_t bright)
{
    ov_write_reg(&ov_dev, 0xff, 0x00);
    ov_write_reg(&ov_dev, 0x7c, 0x00);
    ov_write_reg(&ov_dev, 0x7d, 0x04);
    ov_write_reg(&ov_dev, 0x7c, 0x09);
    ov_write_reg(&ov_dev, 0x7d, (bright + 2) << 4);
    ov_write_reg(&ov_dev, 0x7d, 0x00);
}

// range: [-2, +2]
void ov_contrast(int8_t contrast)
{
    uint8_t reg_7d_0, reg_7d_1;
    switch (contrast) {
    case -2:
        reg_7d_0=0x18;
        reg_7d_1=0x34;
        break;
    case -1:
        reg_7d_0=0x1C;
        reg_7d_1=0x2A;
        break;
    case 1:
        reg_7d_0=0x24;
        reg_7d_1=0x16;
        break;
    case 2:
        reg_7d_0=0x28;
        reg_7d_1=0x0C;
        break;
    default:
        reg_7d_0=0x20;
        reg_7d_1=0x20;
    }
    ov_write_reg(&ov_dev, 0xff, 0x00);
    ov_write_reg(&ov_dev, 0x7c, 0x00);
    ov_write_reg(&ov_dev, 0x7d, 0x04);
    ov_write_reg(&ov_dev, 0x7c, 0x07);
    ov_write_reg(&ov_dev, 0x7d, 0x20);
    ov_write_reg(&ov_dev, 0x7d, reg_7d_0);
    ov_write_reg(&ov_dev, 0x7d, reg_7d_1);
    ov_write_reg(&ov_dev, 0x7d, 0x06);
}


void ov_out_win(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    uint16_t endx;
    uint16_t endy;
    uint8_t temp;
    endx = sx + width / 2;
    endy = sy + height / 2;

    ov_write_reg(&ov_dev, 0xFF, 0x01);
    temp = ov_read_reg(&ov_dev, 0x03);
    temp &= 0xF0;
    temp |= ((endy & 0x03) << 2) | (sy & 0x03);
    ov_write_reg(&ov_dev, 0x03, temp);
    ov_write_reg(&ov_dev, 0x19, sy >> 2);
    ov_write_reg(&ov_dev, 0x1A, endy >> 2);

    temp = ov_read_reg(&ov_dev, 0x32);
    temp &= 0xC0;
    temp |= ((endx & 0x07) << 3) | (sx & 0x07);
    ov_write_reg(&ov_dev, 0x32, temp);
    ov_write_reg(&ov_dev, 0x17, sx >> 3);
    ov_write_reg(&ov_dev, 0x18, endx >> 3);
}

int ov_out_size(uint16_t width,uint16_t height)
{
    uint16_t outh;
    uint16_t outw;
    uint8_t temp;
    if (width % 4 || height % 4)
        return -1;
    outw = width / 4;
    outh = height / 4;
    ov_write_reg(&ov_dev, 0xFF, 0x00);
    ov_write_reg(&ov_dev, 0xE0, 0x04);
    ov_write_reg(&ov_dev, 0x5A, outw & 0xFF);
    ov_write_reg(&ov_dev, 0x5B, outh & 0xFF);
    temp = (outw >> 8) & 0x03;
    temp |= (outh >> 6) & 0x04;
    ov_write_reg(&ov_dev, 0x5C, temp);
    ov_write_reg(&ov_dev, 0xE0, 0x00);
    return 0;
}

int ov_image_win(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height)
{
    uint16_t hsize, vsize;
    uint8_t temp;
    if (width % 4 || height % 4)
        return -1;
    hsize = width / 4;
    vsize = height / 4;
    ov_write_reg(&ov_dev, 0xFF, 0x00);
    ov_write_reg(&ov_dev, 0xE0, 0x04);
    ov_write_reg(&ov_dev, 0x51, hsize & 0xFF);
    ov_write_reg(&ov_dev, 0x52, vsize & 0xFF);
    ov_write_reg(&ov_dev, 0x53, offx & 0xFF);
    ov_write_reg(&ov_dev, 0x54, offy & 0xFF);
    temp = (vsize >> 1) & 0x80;
    temp |= (offy >> 4) & 0x70;
    temp |= (hsize >> 5) & 0x08;
    temp |= (offx >> 8) & 0x07;
    ov_write_reg(&ov_dev, 0x55, temp);
    ov_write_reg(&ov_dev, 0x57, (hsize >> 2) & 0x80);
    ov_write_reg(&ov_dev, 0xE0, 0x00);
    return 0;
} 

#if 0
// UXGA: 1600x1200, SVGA: 800x600, CIF: 352x288
int ov_image_size(uint16_t width, uint16_t height)
{ 
    uint8_t temp;
    ov_write_reg(&ov_dev, 0xFF, 0x00);
    ov_write_reg(&ov_dev, 0xE0, 0x04);
    ov_write_reg(&ov_dev, 0xC0, (width >> 3) & 0xFF);
    ov_write_reg(&ov_dev, 0xC1, (height >> 3) & 0xFF);
    temp = (width & 0x07) << 3;
    temp |= height & 0x07;
    temp |= (width >> 4) & 0x80;
    ov_write_reg(&ov_dev, 0x8C, temp);
    ov_write_reg(&ov_dev, 0xE0, 0x00);
    return 0;
}
#endif

int ov2640_init(void)
{
    gpio_set_value(&ov_rst, 0);
    delay_systick(10000 / SYSTICK_US_DIV);
    gpio_set_value(&ov_rst, 1);
    delay_systick(10000 / SYSTICK_US_DIV);

    ov_write_reg(&ov_dev, 0xff, 0x01);
    ov_write_reg(&ov_dev, OV_SEN_COM7, 0x80); // soft reset
    delay_systick(50000 / SYSTICK_US_DIV);

    uint16_t ov_mid = (ov_read_reg(&ov_dev, OV_SEN_MIDH) << 8) | ov_read_reg(&ov_dev, OV_SEN_MIDL);
    uint16_t ov_pid = (ov_read_reg(&ov_dev, OV_SEN_PIDH) << 8) | ov_read_reg(&ov_dev, OV_SEN_PIDL);
    d_info("ov_mid: %04x, pid: %04x\n", ov_mid, ov_pid);
    if (ov_mid != OV_MID || ov_pid != OV_PID) {
        d_info("ov: wrong mid or pid\n");
        return -1;
    }

    for(int i = 0; i < sizeof(ov_svga_init_tbl) / 2; i++)
        ov_write_reg(&ov_dev, ov_svga_init_tbl[i][0], ov_svga_init_tbl[i][1]);

    ov_jpeg_mode();
    ov_image_win(0, 0, 800, 600);
    ov_out_size(csa.width, csa.height);

    ov_write_reg(&ov_dev, 0xff, 0x00);
    d_info("pclk: %04x\n", ov_read_reg(&ov_dev, 0xd3));
    ov_write_reg(&ov_dev, 0xd3, 0x15);
    d_info("pclk: %04x\n", ov_read_reg(&ov_dev, 0xd3));

    ov_write_reg(&ov_dev, 0xff, 0x01);
    ov_write_reg(&ov_dev, 0x11, 0x01);

    ov_light_mode(3);
    ov_color_saturation(0);
    ov_brightness(0);
    ov_contrast(0);
    if (csa.manual) {
        ov_manual_exposure(csa.exposure);
        ov_manual_agc(csa.agc);
    }
    return 0;
}
