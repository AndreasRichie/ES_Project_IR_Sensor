#include "display_handler.h"

#include <math.h>
#include <ui.h>

#include <vector>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "lv_port.h"

// the colors we will be using for thermal image
const std::vector<uint16_t> camColors = {
    0x480F, 0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810,
    0x3010, 0x3010, 0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010,
    0x2010, 0x1810, 0x1810, 0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811,
    0x0811, 0x0811, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0031, 0x0031,
    0x0051, 0x0072, 0x0072, 0x0092, 0x00B2, 0x00B2, 0x00D2, 0x00F2, 0x00F2,
    0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192, 0x0192, 0x01B2, 0x01D2,
    0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273, 0x0293, 0x02B3,
    0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373, 0x0394,
    0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
    0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554,
    0x0574, 0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571,
    0x0591, 0x0591, 0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE,
    0x05AE, 0x05AD, 0x05AD, 0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB,
    0x05CA, 0x05CA, 0x05CA, 0x05C9, 0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7,
    0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5, 0x05E5, 0x0604, 0x0604, 0x0604,
    0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621, 0x0621, 0x0620, 0x0620,
    0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640, 0x1E40, 0x1E40,
    0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60, 0x3E60,
    0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
    0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0,
    0x8EA0, 0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0,
    0xB6E0, 0xB6E0, 0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0,
    0xD700, 0xDF00, 0xDEE0, 0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640,
    0xE620, 0xE600, 0xE5E0, 0xE5C0, 0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520,
    0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480, 0xE460, 0xEC40, 0xEC20, 0xEC00,
    0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40, 0xEB20, 0xEB00, 0xEAE0,
    0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200, 0xF1E0, 0xF1C0,
    0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0, 0xF080,
    0xF060, 0xF040, 0xF020, 0xF800,
};

display_handler::display_handler()
    : tag("handle_display_task"), camera_label("Heat Image of IR Camera ") {}

display_handler::~display_handler() = default;

void display_handler::init_display() {
  lv_port_init();
  ui_init();
  handle_screen(0);
}

void display_handler::handle_screen(const int display_index) {
  if (display_index < 6)
    handle_camera_screen_load(display_index);
  else if (display_index == 6)
    handle_values_screen_load();
  else
    ESP_LOGE(tag.c_str(), "An invalid display_index occured!");
}

void display_handler::handle_values(const measurement_data& sensor_data,
                                    const int display_index) const {
  if (display_index < 6)
    handle_camera_screen(sensor_data, display_index);
  else if (display_index == 6)
    handle_values_screen(sensor_data);
  else
    ESP_LOGE(tag.c_str(), "An invalid display_index occured!");
  vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
}

void display_handler::handle_camera_screen(const measurement_data& sensor_data,
                                           const int camera_index) const {
  for (unsigned int index = 0; index < PIXEL_COUNT; ++index) {
    handle_pixel(camera_index, index, sensor_data);
  }
}

lv_color16_t display_handler::get_pixel_color(const float value) const {
  auto pixel_value = value;
  if (pixel_value >= MAXTEMP)
    pixel_value = MAXTEMP;
  else if (pixel_value <= MINTEMP)
    pixel_value = MINTEMP;

  uint8_t colorIndex =
      std::round(camColors.size() / (MAXTEMP - MINTEMP) * pixel_value -
                 camColors.size() / ((float)MAXTEMP / (float)MINTEMP - 1));
  lv_color16_t color;
  color.full = camColors[colorIndex];
  return color;
}

void display_handler::handle_pixel(const int camera_index,
                                   const unsigned int pixel_index,
                                   const measurement_data& sensor_data) const {
  const auto pixel_object = lv_obj_get_child(ui_PixelGrid, pixel_index);
  lv_obj_t* button_label;
  if (lv_obj_get_child_cnt(pixel_object) == 0)
    button_label = lv_label_create(pixel_object);
  else
    button_label = lv_obj_get_child(pixel_object, 0);
  auto pixel_value = sensor_data.ir_camera_data[camera_index][pixel_index];
  lv_label_set_text_fmt(button_label, "%.2f", pixel_value);
  lv_obj_center(button_label);

  lv_obj_set_style_bg_color(pixel_object, get_pixel_color(pixel_value),
                            LV_PART_MAIN);
}

void display_handler::handle_values_screen(
    const measurement_data& sensor_data) const {
  lv_label_set_text_fmt(ui_Value, "%.2f", sensor_data.surface_temp);
  lv_label_set_text_fmt(ui_Value1, "%.2f", sensor_data.air_temp);
  lv_label_set_text_fmt(ui_Value2, "%.2f", sensor_data.air_rH);
}

void display_handler::handle_camera_screen_load(const int camera_index) {
  lv_disp_load_scr(ui_ScreenCamera);
  // lv_obj_set_style_transform_pivot_x(ui_PixelGrid, 200, NULL);
  // lv_obj_set_style_transform_pivot_y(ui_PixelGrid, 200, NULL);
  // lv_obj_set_style_transform_angle(ui_PixelGrid, 2700, NULL);
  lv_label_set_text_fmt(ui_TitleCamera, "%s%d", camera_label.c_str(),
                        camera_index + 1);
}

void display_handler::handle_values_screen_load() {
  lv_disp_load_scr(ui_ScreenSensors);
}
