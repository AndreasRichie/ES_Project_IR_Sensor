#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#pragma once

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "lvgl.h"
#include "measurement_data.h"

#define MINTEMP 15
#define MAXTEMP 35
#define DISPLAY_INTERVAL_MS 100

#define DISPLAY_QUEUE_SIZE 20

// todo indicate that lorawan join successful

class Display_Handler {
 private:
  Display_Handler() = default;
  ~Display_Handler() = default;

 public:
  Display_Handler(Display_Handler& other) = delete;  // not clonable
  void operator=(const Display_Handler&) = delete;   // not assignable
  static Display_Handler* get_instance();

 public:
  void init_display();
  void add_to_queue(const display_data& data);
  void set_joined_icon(const bool joined);

 private:
  static void handle_display_task_wrapper(void* pvParameters);
  void Handle_Display_Task();

 private:
  void create_button_labels();
  void init_pixels();
  void init_clock_icon();
  void init_calibrated_icons();
  void init_joined_icon();
  bool is_pixel_clicked(const lv_obj_t* clicked) const;
  void add_callbacks();
  tm get_default_time() const;

  void handle_values(const display_data& values);
  void handle_surface_temp_value(const float surface_temp);
  void handle_pixel_values(const uint8_t index, const pixel_values_t& pixel_values);
  void handle_calibration_values(const uint8_t index, const pixel_values_t& calibration_values);

  bool check_display_index_camera(const std::string& caller) const;
  lv_color16_t get_pixel_color(const float value) const;
  void handle_pixel(const unsigned int pixel_index) const;
  void handle_camera_screen_load();
  void handle_values_screen_load();
  void handle_lora_settings_screen_load();
  std::string get_join_button_string(const bool state) const;
  void handle_join_display(const bool is_join) const;
  void handle_time_settings_screen_load();
  void handle_calibration_view();

  void check_if_time_set();
  void handle_screen();
  void handle_temp_sensor_option() const;
  void handle_calibrated_icon();

  enum calibration_type { manual, sensor };

  void join_button_pressed_cb();
  void swipe_event_cb();
  void time_save_button_pressed_cb();
  void date_change_cb();
  void calibration_check_cb(lv_event_t* e);
  void calibration_choice_cb(lv_event_t* e);
  void select_all_cb();
  void confirm_pixels_cb();
  void confirm_calibration_cb();
  void cancel_calibration_cb();
  void calibration_choice_cb();

 private:
  const std::string m_tag{"Display_Handler"};
  int m_display_index{0};
  bool m_display_lora_settings{false};
  std::optional<tm> m_current_date_time;
  bool m_display_time_settings{false};
  std::array<lv_obj_t*, PIXEL_COUNT> m_pixels{};
  lv_obj_t* m_clock_icon{NULL};
  bool m_calibration_active{false};
  display_values m_display_values;
  std::array<bool, NUMBER_OF_CAMERAS> m_calibration_set{false};
  lv_obj_t* m_calibrated_red_icon{NULL};
  lv_obj_t* m_calibrated_green_icon{NULL};
  QueueHandle_t m_display_queue{NULL};
  lv_obj_t* m_joined_icon{NULL};
};

#endif