SET(SOURCES components/ui_comp_buttonminus.c
    components/ui_comp.c
    screens/ui_ScreenCamera.c
    screens/ui_ScreenSensors.c
    screens/ui_ScreenLoRa.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_location2_png.c
    fonts/ui_font_Font1.c)

add_library(ui ${SOURCES})
