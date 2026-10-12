idf_component_register(
    SRCS
        "main.c"
        "ns2_proto.c"
        "ble_central.c"
        "bonds.c"
        "pad.c"
        "usb_gamepad.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        esp_timer
        esp_driver_gpio
        esp_tinyusb
)
