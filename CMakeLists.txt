cmake_minimum_required(VERSION 3.16)

# Normal builds fetch esp_tinyusb from the ESP component registry (see
# main/idf_component.yml). Offline builds can point DONGLE_EXTRA_COMPONENTS at a
# directory holding local copies of `esp_tinyusb` and `tinyusb` instead, and set
# IDF_COMPONENT_MANAGER=0.
if(DEFINED ENV{DONGLE_EXTRA_COMPONENTS})
    set(EXTRA_COMPONENT_DIRS $ENV{DONGLE_EXTRA_COMPONENTS})
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gamestation_dongle)
