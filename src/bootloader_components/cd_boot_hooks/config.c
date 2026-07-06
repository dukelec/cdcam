#include "hooks.h"
#include "modbus_crc.h"

const csa_sm_t csa_dft = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,
        .bus_cfg = CDCTL_CFG_DFT(0xfe)
};

csa_t csa = {0};

uint8_t fbuf[4096] __attribute__((aligned(4)));

_Static_assert(offsetof(csa_t, _end_save) == 1024, "csa size err");


int flash_cal_crc(uint32_t src_addr, uint32_t len, uint16_t *crc)
{
    uint16_t c = 0xffff;
    uint32_t p = src_addr;
    while (true) {
        uint32_t left_len = len - (p - src_addr);
        uint32_t sub_len = min(left_len, 4096);
        if (sub_len == 0)
            break;
        uint32_t copy_len = (sub_len + 3) & ~3;
        int ret = esp_rom_spiflash_read(p, (void *)fbuf, copy_len);
        if (ret != 0)
            return ret;
        c = crc16_sub(fbuf, sub_len, c);
        p += sub_len;
    }
    *crc = c;
    return 0;
}


int flash_move(uint32_t src_addr, uint32_t dst_addr, uint32_t len)
{
    uint32_t p = src_addr;
    while (true) {
        uint32_t left_len = len - (p - src_addr);
        uint32_t sub_len = min(left_len, 4096);
        if (sub_len == 0)
            break;
        uint32_t copy_len = (sub_len + 3) & ~3;
        int ret = esp_rom_spiflash_read(p, (void *)fbuf, copy_len);
        if (ret != 0)
            return ret;
        ret = esp_rom_spiflash_write(dst_addr + (p - src_addr), (void *)fbuf, copy_len);
        if (ret != 0)
            return ret;
        p += sub_len;
    }
    return 0;
}


// out buf size >= (len + 3) & ~3
int flash_read(uint32_t addr, uint8_t *out, uint32_t len)
{
    if (addr & 3)
        return -1;
    int ret = esp_rom_spiflash_read(addr, (void *)out, (len + 3) & ~3);
    return ret;
}

int flash_erase(uint32_t addr, uint32_t len)
{
    uint32_t _addr = addr & ~4095;
    uint32_t _len = len + (addr - _addr);
    if (_addr < BL_FLASH_PROTECT)
        return -1;
    for (uint32_t i = 0; i < (_len + 4095) / 4096; i++) {
        int ret = esp_rom_spiflash_erase_sector(_addr / 4096 + i);
        if (ret)
            return ret;
    }
    return 0;
}

int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf)
{
    if ((addr & 3) || addr < BL_FLASH_PROTECT)
        return -1;
    return esp_rom_spiflash_write(addr, (void *)buf, (len + 3) & ~3);
}


void load_conf(void)
{
    int ret = flash_read(APP_CONF_ADDR, (uint8_t *)&csa, offsetof(csa_t, _end_save));

    if (ret == 0 && csa.magic_code == 0xcdcd && (csa.conf_ver >> 8) == (APP_CONF_VER >> 8)) {
        csa.conf_from = 1;
    } else {
        memset(&csa, 0, sizeof(csa_t));
        memcpy(&csa, &csa_dft, sizeof(csa_dft));
    }
    memset(&csa.do_reboot, 0, 3); // clear do_reboot, keep_in_bl, save_conf
    d_info("bl: conf_from %d, mac %02x\n", csa.conf_from, csa.bus_cfg.mac);
}

int save_conf(void)
{
    int ret = flash_erase(APP_CONF_ADDR, 4096);
    ret |= flash_write(APP_CONF_ADDR, offsetof(csa_t, _end_save), (uint8_t *)&csa);
    d_info("bl: save conf %s\n", ret ? "err" : "ok");
    return ret;
}
