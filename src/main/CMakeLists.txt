
idf_component_register(
    SRCS
        "main.c"
        "sensor_init.c"
        "cdnet/arch/esp32_riscv/arch_wrapper.c"
        "cdnet/utils/cd_list.c"
        "cdnet/utils/modbus_crc.c"
        "cdnet/dev/cdctl_pll_cal.c"
        "cdnet/dev/cdctl_it.c"
        "usr/cd_main.c"
        "usr/comm_service.c"
        "usr/config.c"

    INCLUDE_DIRS
        "."
        "usr"
        "cdnet/arch/esp32_riscv"
        "cdnet/utils"
        "cdnet/dev"

    REQUIRES
        esp_mm
        esp_driver_jpeg
        esp_driver_isp
        esp_driver_cam
        esp_driver_i2c
        esp_driver_spi
        nvs_flash
        spi_flash
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_ppa
        esp_cam_sensor
)
