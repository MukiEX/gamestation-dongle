idf_component_register(
    SRCS
        "main.c"
        "ns2_proto.c"
        "ble_central.c"
        "bonds.c"
        "diag.c"
        "pad.c"
        "usb_gamepad.c"
        "settings.c"
        "rumble.c"
        "out_hid.c"
        "out_xinput.c"
        "out_ds4.c"
        "out_switchpro.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        esp_timer
        esp_driver_gpio
        esp_tinyusb
        espcoredump
)

# Diagnostic build: `DONGLE_DIAG=1` in the environment when configuring.
if(DEFINED ENV{DONGLE_DIAG})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DONGLE_DIAG=1)
endif()
