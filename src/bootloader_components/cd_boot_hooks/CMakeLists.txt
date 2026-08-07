
idf_component_register(
    SRCS
        "hooks.c"
        "config.c"
        "comm_service.c"
        "cdctl_spi_wr.c"
        "cdnet/utils/modbus_crc.c"
        "cdnet/utils/cd_list.c"
        "cdnet/utils/hex_dump.c"
        "cdnet/dev/cdctl.c"
        "cdnet/dev/cdctl_pll_cal.c"

    INCLUDE_DIRS
        "."
        "cdnet/dev"
        "cdnet/utils"
        "cdnet/arch/esp32_riscv"

    PRIV_REQUIRES
        hal
        soc
        log
        esp_rom
        esp_hal_gpio
        esp_hal_clock
        esp_hal_wdt
)
