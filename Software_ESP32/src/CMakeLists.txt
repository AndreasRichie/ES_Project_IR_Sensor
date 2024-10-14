set(UI_DIR ../lib/hmi_ui)
file(GLOB_RECURSE UI_SOURCES ${UI_DIR}/*.c)

set(UTIL_DIR ../lib/util)
file(GLOB_RECURSE UTIL_SOURCES ${UTIL_DIR}/*.c)

idf_component_register(
    SRCS "main.cpp" "lv_port.c" "measurement_data.cpp" "display_handler.cpp" "uart_handler.cpp" "lorawan_handler.cpp" ${UI_SOURCES} ${UTIL_SOURCES}
    INCLUDE_DIRS "."  ${UI_DIR} ${UTIL_DIR}
    REQUIRES bootloader_support)
