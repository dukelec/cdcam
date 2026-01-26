
idf_component_register(
    SRCS
        "main.c"
        "sensor_init.c"
        "esp_cam/esp_cam_sensor.c"
        "esp_cam/ov5647.c"
        "utils/cd_list.c"
        "utils/cdctl_pll_cal.c"
        "utils/cdctl_it.c"
        "utils/modbus_crc.c"
        "usr/cd_main.c"
        "usr/cdctl_spi_wr.c"
        "usr/common_services.c"
        "usr/config.c"

    INCLUDE_DIRS
        "."
        "usr"
        "utils"
        "esp_cam"

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

    LDFRAGMENTS
        linker.lf
)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ov5647_detect")
