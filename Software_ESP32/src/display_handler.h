#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#pragma once

#include "lvgl.h"
#include "measurement_data.h"

#define MINTEMP 20
#define MAXTEMP 35
#define DISPLAY_INTERVAL_MS 100

class display_handler {
 public:
  display_handler();
  ~display_handler();
  void init_display();
  void handle_values(const measurement_data& sensor_data) const;

 private:
  void create_button_labels();
  void add_callbacks();

  void handle_camera_screen(const measurement_data& sensor_data,
                            const int camera_index) const;
  lv_color16_t get_pixel_color(const float value) const;
  void handle_pixel(const int camera_index, const unsigned int pixel_index,
                    const measurement_data& sensor_data) const;
  void handle_values_screen(const measurement_data& sensor_data) const;
  void handle_camera_screen_load(const int camera_index);
  void handle_values_screen_load();
  void handle_lora_settings_screen_load();
  std::string get_join_button_string() const;

  void handle_screen();

  void join_button_pressed_cb(lv_event_t* e);
  void swipe_event_cb(lv_event_t* e);

 private:
  const std::string tag;
  const std::string camera_label;
  int display_index;
  bool display_lora_settings;
  bool is_join;
};

#endif