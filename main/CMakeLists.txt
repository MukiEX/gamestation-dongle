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
        "out_ds4.c" "out_ds5.c" "out_steam.c" "out_touch.c"
        "out_switchpro.c"
        "config_server.c"
        "display.c"
        "hid_gamepad.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "web/index.html"
    REQUIRES
        bt
        nvs_flash
        esp_timer
        esp_driver_gpio
        esp_tinyusb
        espcoredump
        esp_netif
        esp_event
        esp_http_server
        json
        lwip
        esp_lcd
        esp_driver_spi
        esp_driver_ledc
)

# esp_tinyusb owns the string-descriptor callback; wrapping it lets the
# dongle add the Microsoft OS string (0xEE) for the generic XInput ID.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=tud_descriptor_string_cb")

# Board: `DONGLE_BOARD=tdongle` in the environment when first configuring
# builds for the LILYGO T-Dongle-S3 (display + RGB LED). Kept in the cache.
if(DEFINED ENV{DONGLE_BOARD})
    set(DONGLE_BOARD_NAME "$ENV{DONGLE_BOARD}" CACHE STRING "Board" FORCE)
endif()
if(DONGLE_BOARD_NAME STREQUAL "tdongle")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DONGLE_BOARD_TDONGLE=1)
endif()

# Diagnostic build: `DONGLE_DIAG=1` in the environment when first configuring.
# Kept in the CMake cache, so an automatic re-configure (new source files...)
# without the variable doesn't silently turn it back into a normal build.
if(DEFINED ENV{DONGLE_DIAG})
    set(DONGLE_DIAG_BUILD ON CACHE BOOL "Diagnostic build" FORCE)
endif()
if(DONGLE_DIAG_BUILD)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DONGLE_DIAG=1)
endif()
