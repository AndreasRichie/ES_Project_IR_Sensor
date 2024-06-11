cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS  ./components)

set(EXCLUDE_COMPONENTS LoRaWAN)

add_compile_options(-fdiagnostics-color=always -w)

project(Software_ESP32)
