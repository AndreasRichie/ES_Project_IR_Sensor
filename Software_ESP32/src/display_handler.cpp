#include "display_handler.h"

#include <math.h>
#include <ui.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>

#include "esp_log.h"
#include "lorawan_handler.h"
#include "lv_port.h"
#include "priorities.h"
#include "uart_handler.h"

// the colors we will be using for thermal image
const std::vector<uint16_t> camColors = {
    0x480F, 0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010, 0x3010, 0x2810, 0x2810,
    0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810, 0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811,
    0x0811, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2, 0x00B2,
    0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192, 0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3,
    0x0213, 0x0233, 0x0253, 0x0253, 0x0273, 0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353,
    0x0373, 0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474, 0x0494, 0x04B4, 0x04D4,
    0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574, 0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572,
    0x0571, 0x0591, 0x0591, 0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD, 0x05AD,
    0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9, 0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7,
    0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5, 0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601,
    0x0621, 0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640, 0x1E40, 0x1E40, 0x2640,
    0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60, 0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680,
    0x5E80, 0x5E80, 0x6680, 0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0, 0x8EC0,
    0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0, 0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0,
    0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0, 0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0,
    0xE5C0, 0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480, 0xE460, 0xEC40, 0xEC20,
    0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40, 0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60,
    0xEA40, 0xF220, 0xF200, 0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0, 0xF080,
    0xF060, 0xF040, 0xF020, 0xF800,
};

Display_Handler* Display_Handler::get_instance() {
  static Display_Handler instance;
  return &instance;
}

void Display_Handler::init_display() {
  m_display_queue = xQueueCreate(DISPLAY_QUEUE_SIZE, sizeof(display_data));
  if (m_display_queue == NULL) {
    ESP_LOGE(m_tag.c_str(), "Could not create queue!");
    ESP_ERROR_CHECK(ESP_FAIL);
  }
  lv_port_init();
  ui_init();
  init_pixels();
  init_clock_icon();
  init_calibrated_icons();
  init_joined_icon();
  create_button_labels();
  add_callbacks();
  handle_screen();
  xTaskCreate(handle_display_task_wrapper, "Handle_Display_Task", 1024 * 12, NULL, DISPLAY_PRIORITY, NULL);
}

void Display_Handler::add_to_queue(const display_data& data) {
  if (m_display_queue == NULL) {
    ESP_LOGE(m_tag.c_str(), "Queue is NULL!");
    return;
  }
  if (uxQueueSpacesAvailable(m_display_queue) == 0) {
    display_data data_old;
    xQueueReceive(m_display_queue, &data_old, portMAX_DELAY);
  }
  xQueueSendToBack(m_display_queue, &data, portMAX_DELAY);
}

void Display_Handler::set_joined_icon(const bool joined) {
  if (m_joined_icon == NULL) return;
  if (joined)
    lv_obj_clear_flag(m_joined_icon, LV_OBJ_FLAG_HIDDEN);
  else
    lv_obj_add_flag(m_joined_icon, LV_OBJ_FLAG_HIDDEN);
}

void Display_Handler::handle_display_task_wrapper([[maybe_unused]] void* pvParameters) {
  Display_Handler::get_instance()->Handle_Display_Task();
}

void Display_Handler::Handle_Display_Task() {
  display_data data;
  while (1) {
    if (xQueueReceive(m_display_queue, static_cast<void*>(&data), portMAX_DELAY) == pdTRUE) {
      handle_values(data);
    } else {
      ESP_LOGE(m_tag.c_str(), "xQueueReceive returned pdFALSE!");
    }
  }
}

void Display_Handler::create_button_labels() {
  for (lv_obj_t* pixel : m_pixels) lv_label_create(pixel);
}

void Display_Handler::init_pixels() {
  for (std::size_t index = 0; index < PIXEL_COUNT; ++index) m_pixels[index] = lv_obj_get_child(ui_PixelGrid, index);
}

void Display_Handler::init_clock_icon() {
  LV_IMG_DECLARE(ui_img_clock_png);  // assets/clock.png
  m_clock_icon = lv_img_create(lv_disp_get_layer_top(NULL));
  lv_img_set_src(m_clock_icon, &ui_img_clock_png);
  lv_img_set_zoom(m_clock_icon, 230);
  lv_obj_align(m_clock_icon, LV_ALIGN_BOTTOM_LEFT, 4, -4);
}

void Display_Handler::init_calibrated_icons() {
  LV_IMG_DECLARE(ui_img_calibration_red_png);  // assets/calibration_red.png
  m_calibrated_red_icon = lv_img_create(ui_ScreenCamera);
  lv_img_set_src(m_calibrated_red_icon, &ui_img_calibration_red_png);
  lv_obj_align(m_calibrated_red_icon, LV_ALIGN_BOTTOM_RIGHT, -4, -45);
  LV_IMG_DECLARE(ui_img_calibration_green_png);  // assets/calibration_green.png
  m_calibrated_green_icon = lv_img_create(ui_ScreenCamera);
  lv_img_set_src(m_calibrated_green_icon, &ui_img_calibration_green_png);
  lv_obj_align(m_calibrated_green_icon, LV_ALIGN_BOTTOM_RIGHT, -4, -45);
  lv_obj_add_flag(m_calibrated_green_icon, LV_OBJ_FLAG_HIDDEN);
}

void Display_Handler::init_joined_icon() {
  LV_IMG_DECLARE(ui_img_antenna_png);  // assets/antenna.png
  m_joined_icon = lv_img_create(lv_disp_get_layer_top(NULL));
  lv_img_set_src(m_joined_icon, &ui_img_antenna_png);
  // lv_img_set_zoom(m_joined_icon, 230);
  lv_obj_align(m_joined_icon, LV_ALIGN_BOTTOM_RIGHT, -4, -4);
  lv_obj_add_flag(m_joined_icon, LV_OBJ_FLAG_HIDDEN);
}

bool Display_Handler::is_pixel_clicked(const lv_obj_t* clicked) const {
  return std::any_of(m_pixels.cbegin(), m_pixels.cend(), [clicked](const lv_obj_t* pixel) { return pixel == clicked; });
}

void Display_Handler::add_callbacks() {
  auto event_cb = [](lv_event_t* e) {
    auto handler = static_cast<Display_Handler*>(lv_event_get_user_data(e));
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* clicked_element = lv_event_get_current_target(e);

    switch (code) {
      case LV_EVENT_CLICKED:
        if (clicked_element == ui_ButtonJoin)
          handler->join_button_pressed_cb();
        else if (clicked_element == ui_ButtonSave)
          handler->time_save_button_pressed_cb();
        else if (clicked_element == ui_ButtonOk)
          handler->confirm_pixels_cb();
        else if (clicked_element == ui_ButtonAll)
          handler->select_all_cb();
        else if (clicked_element == ui_ButtonCancel)
          handler->cancel_calibration_cb();
        else if (clicked_element == ui_ButtonConfirm)
          handler->confirm_calibration_cb();
        break;
      case LV_EVENT_SHORT_CLICKED:
        if (handler->is_pixel_clicked(clicked_element)) handler->calibration_choice_cb(e);
        break;
      case LV_EVENT_LONG_PRESSED:
        handler->calibration_check_cb(e);
        break;
      case LV_EVENT_GESTURE:
        handler->swipe_event_cb();
        break;
      case LV_EVENT_VALUE_CHANGED:
        if (clicked_element == ui_Date)
          handler->date_change_cb();
        else if (clicked_element == ui_CalibOption)
          handler->calibration_choice_cb();
        break;
      default:
        break;
    }
  };

  lv_obj_add_event_cb(ui_ScreenCamera, event_cb, LV_EVENT_GESTURE, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ScreenSensors, event_cb, LV_EVENT_GESTURE, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ScreenLoRa, event_cb, LV_EVENT_GESTURE, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ScreenTime, event_cb, LV_EVENT_GESTURE, static_cast<void*>(this));

  lv_obj_add_event_cb(ui_ButtonJoin, event_cb, LV_EVENT_CLICKED, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ButtonSave, event_cb, LV_EVENT_CLICKED, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ButtonOk, event_cb, LV_EVENT_CLICKED, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ButtonAll, event_cb, LV_EVENT_CLICKED, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ButtonConfirm, event_cb, LV_EVENT_CLICKED, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_ButtonCancel, event_cb, LV_EVENT_CLICKED, static_cast<void*>(this));

  for (lv_obj_t* pixel : m_pixels) lv_obj_add_event_cb(pixel, event_cb, LV_EVENT_ALL, static_cast<void*>(this));

  lv_obj_add_event_cb(ui_Date, event_cb, LV_EVENT_VALUE_CHANGED, static_cast<void*>(this));
  lv_obj_add_event_cb(ui_CalibOption, event_cb, LV_EVENT_VALUE_CHANGED, static_cast<void*>(this));
}

tm Display_Handler::get_default_time() const {
  std::string date_string = __DATE__;
  std::string time_string = __TIME__;

  std::istringstream date_stream(date_string);
  std::string string_month;
  int day;
  int year;
  date_stream >> string_month >> day >> year;

  int month;
  if (string_month == "Jan")
    month = 1;
  else if (string_month == "Feb")
    month = 2;
  else if (string_month == "Mar")
    month = 3;
  else if (string_month == "Apr")
    month = 4;
  else if (string_month == "May")
    month = 5;
  else if (string_month == "Jun")
    month = 6;
  else if (string_month == "Jul")
    month = 7;
  else if (string_month == "Aug")
    month = 8;
  else if (string_month == "Sep")
    month = 9;
  else if (string_month == "Oct")
    month = 10;
  else if (string_month == "Nov")
    month = 11;
  else if (string_month == "Dec")
    month = 12;
  else {
    ESP_LOGE(m_tag.c_str(), "An invalid month occured in __DATE__!");
    return tm{};
  }

  for (std::string::size_type pos = time_string.find(':'); pos != std::string::npos; pos = time_string.find(':', pos))
    time_string[pos] = ' ';
  std::istringstream time_stream(time_string);
  int hour, min, sec;
  time_stream >> hour >> min >> sec;

  tm default_date_time;
  default_date_time.tm_mon = month - 1;
  default_date_time.tm_mday = day;
  default_date_time.tm_year = year - 1900;
  default_date_time.tm_hour = hour;
  default_date_time.tm_min = min;
  default_date_time.tm_sec = sec;
  default_date_time.tm_isdst = -1;
  return default_date_time;
}

void Display_Handler::handle_values(const display_data& values) {
  value_type type = static_cast<value_type>(values.type);

  switch (type) {
    case value_type::pixel_values:
      handle_pixel_values(values.index, values.data);
      break;
    case value_type::surface_temperature:
      handle_surface_temp_value(values.data[0]);
      break;
    case value_type::air_temperature:
      m_display_values.air_temperature = values.data[0];
      break;
    case value_type::air_humidity:
      m_display_values.air_humidity = values.data[0];
      break;
    case value_type::mean_radiant_temperature:
      m_display_values.mean_radiant_temperature = values.data[0];
      break;
    case value_type::calibration:
      handle_calibration_values(values.index, values.data);
      break;
    default:
      ESP_LOGE(m_tag.c_str(), "An unhandled value_type occured!");
      break;
  }

  if ((values.index == m_display_index) && !m_display_lora_settings && !m_display_time_settings) handle_screen();
}

void Display_Handler::handle_surface_temp_value(const float surface_temp) {
  if (surface_temp > 0.f)
    m_display_values.surface_temperature = surface_temp;
  else
    m_display_values.surface_temperature.reset();
  handle_temp_sensor_option();
}

void Display_Handler::handle_pixel_values(const uint8_t index, const pixel_values_t& pixel_values) {
  if (std::any_of(pixel_values.cbegin(), pixel_values.cend(),
                  [](const float pixel_value) { return pixel_value > 0.f; }))
    m_display_values.all_ir_values[index] = pixel_values;
  else
    m_display_values.all_ir_values[index].reset();
}

void Display_Handler::handle_calibration_values(const uint8_t index, const pixel_values_t& calibration_values) {
  if ((m_display_values.calibration_values[index] == calibration_values) || m_calibration_set[index]) return;
  m_display_values.calibration_values[index] = calibration_values;
  handle_calibrated_icon();
}

bool Display_Handler::check_display_index_camera(const std::string& caller) const {
  if (m_display_index < 6 || m_display_index >= 0) return true;
  ESP_LOGE(m_tag.c_str(),
           "%s was called with an invalid camera/display index! "
           "index: %d",
           caller.c_str(), m_display_index);
  return false;
}

lv_color16_t Display_Handler::get_pixel_color(const float value) const {
  auto pixel_value = value;
  if (pixel_value >= MAXTEMP)
    pixel_value = MAXTEMP;
  else if (pixel_value <= MINTEMP)
    pixel_value = MINTEMP;

  uint8_t colorIndex = std::round(((float)camColors.size() - 1) / (float)(MAXTEMP - MINTEMP) * pixel_value -
                                  ((float)camColors.size() - 1) / ((float)MAXTEMP / (float)MINTEMP - 1));

  lv_color16_t color;
  color.full = camColors[colorIndex];
  return color;
}

void Display_Handler::handle_pixel(const unsigned int pixel_index) const {
  if (!check_display_index_camera("handle_pixel")) return;
  const auto pixel_object = m_pixels[pixel_index];
  lv_obj_t* button_label = lv_obj_get_child(pixel_object, 0);
  optional_pixels_t pixel_values = m_display_values.all_ir_values[m_display_index];
  if ((button_label == NULL) || !pixel_values.has_value()) return;
  float pixel_value = pixel_values.value()[pixel_index];
  lv_label_set_text_fmt(button_label, "%.1f", pixel_value);
  lv_obj_center(button_label);
  lv_obj_set_style_bg_color(pixel_object, get_pixel_color(pixel_value), LV_PART_MAIN);
}

void Display_Handler::handle_camera_screen_load() {
  if (!check_display_index_camera("handler_camera_screen_load")) return;
  for (unsigned int index = 0; index < PIXEL_COUNT; ++index) handle_pixel(index);
  if (m_calibration_active) return;
  lv_disp_load_scr(ui_ScreenCamera);
  // m_calibration_active = false;
  handle_calibrated_icon();
  handle_calibration_view();
}

void Display_Handler::handle_values_screen_load() {
  lv_label_set_text_fmt(ui_ValueST, "%.1f", m_display_values.surface_temperature.value_or(0.f));
  lv_label_set_text_fmt(ui_ValueAT, "%.1f", m_display_values.air_temperature);
  lv_label_set_text_fmt(ui_ValueAH, "%.1f", m_display_values.air_humidity);
  lv_label_set_text_fmt(ui_ValueMRT, "%.1f", m_display_values.mean_radiant_temperature);
  lv_disp_load_scr(ui_ScreenSensors);
}

std::string vector_to_string(const std::vector<uint8_t>& data) {
  std::stringstream string_result;
  string_result << std::hex << std::setfill('0');
  for (const auto& element : data) {
    string_result << std::setw(2) << static_cast<unsigned int>(element);
  }
  return string_result.str();
}

void Display_Handler::handle_lora_settings_screen_load() {
  lv_disp_load_scr(ui_ScreenLoRa);
  const auto config = Lorawan_Handler::get_instance()->get_lorawan_config();
  handle_join_display(config.join);
  lv_label_set_text(ui_ValueDevEUI,
                    vector_to_string(std::vector<uint8_t>{config.dev_eui.begin(), config.dev_eui.end()}).c_str());
  lv_label_set_text(ui_ValueJoinEUI,
                    vector_to_string(std::vector<uint8_t>{config.join_eui.begin(), config.join_eui.end()}).c_str());
  lv_label_set_text(ui_ValueAppKey,
                    vector_to_string(std::vector<uint8_t>{config.app_key.begin(), config.app_key.end()}).c_str());
  lv_spinbox_set_value(ui_ValueInterval, static_cast<int32_t>(config.uplink_interval_min));
}

std::string Display_Handler::get_join_button_string(const bool state) const {
  return state ? "Stop Join" : "Start Join";
}

void Display_Handler::handle_join_display(const bool is_join) const {
  if (is_join) {
    /* Disable buttons */
    lv_obj_add_state(ui_ButtonMinus, LV_STATE_DISABLED);
    lv_obj_add_state(ui_ButtonPlus, LV_STATE_DISABLED);
  } else {
    /* Enable buttons */
    lv_obj_clear_state(ui_ButtonMinus, LV_STATE_DISABLED);
    lv_obj_clear_state(ui_ButtonPlus, LV_STATE_DISABLED);
  }
  lv_label_set_text(ui_LabelJoin, get_join_button_string(is_join).c_str());
  lv_obj_center(ui_LabelJoin);
}

void Display_Handler::handle_time_settings_screen_load() {
  lv_disp_load_scr(ui_ScreenTime);
  tm date_time = m_current_date_time.value_or(get_default_time());
  lv_label_set_text_fmt(ui_HoursText, "%02d", date_time.tm_hour);
  lv_label_set_text_fmt(ui_MinutesText, "%02d", date_time.tm_min);
  lv_arc_set_value(ui_HoursSlider, date_time.tm_hour);
  lv_arc_set_value(ui_MinutesSlider, date_time.tm_min);
  lv_calendar_set_today_date(ui_Date, date_time.tm_year + 1900, date_time.tm_mon + 1, date_time.tm_mday);
  lv_calendar_set_showed_date(ui_Date, date_time.tm_year + 1900, date_time.tm_mon + 1);
}

void Display_Handler::handle_calibration_view() {
  std::string description = m_calibration_active ? "Calibrate" : "Heat Image of";
  lv_label_set_text_fmt(ui_TitleCamera, "%s IR Camera %d", description.c_str(), m_display_index + 1);
  if (m_calibration_active) {
    lv_obj_clear_flag(ui_ButtonOk, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ButtonAll, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(ui_ButtonOk, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_ButtonAll, LV_OBJ_FLAG_HIDDEN);
    for (lv_obj_t* pixel : m_pixels) lv_obj_clear_state(pixel, LV_STATE_CHECKED);
  }
}

void Display_Handler::check_if_time_set() {
  if (m_current_date_time.has_value())
    if (m_clock_icon != NULL) lv_obj_add_flag(m_clock_icon, LV_OBJ_FLAG_HIDDEN);
}

void Display_Handler::handle_screen() {
  if (m_display_lora_settings)
    handle_lora_settings_screen_load();
  else if (m_display_time_settings)
    handle_time_settings_screen_load();
  else if (m_display_index < 6)
    handle_camera_screen_load();
  else if (m_display_index == 6)
    handle_values_screen_load();
  else {
    ESP_LOGE(m_tag.c_str(), "An invalid display_index occured!");
    return;
  }
}

void Display_Handler::handle_temp_sensor_option() const {
  if (!m_display_values.surface_temperature.has_value()) {
    if (lv_dropdown_get_option_cnt(ui_CalibOption) != 1)
      lv_dropdown_set_options(ui_CalibOption, "Enter value manually");
    return;
  }
  if (lv_dropdown_get_option_cnt(ui_CalibOption) != 2)
    lv_dropdown_set_options(ui_CalibOption, "Enter value manually\nTemperature sensor");
}

void Display_Handler::handle_calibrated_icon() {
  if (m_display_index > 5) return;
  if ((m_calibrated_green_icon == NULL) || (m_calibrated_red_icon == NULL)) return;
  if (std::all_of(m_display_values.calibration_values[m_display_index].cbegin(),
                  m_display_values.calibration_values[m_display_index].cend(),
                  [](const float value) { return value == 0.f; })) {
    lv_obj_add_flag(m_calibrated_green_icon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(m_calibrated_red_icon, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(m_calibrated_red_icon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(m_calibrated_green_icon, LV_OBJ_FLAG_HIDDEN);
  }
}

void Display_Handler::join_button_pressed_cb() {
  bool join_state = Lorawan_Handler::get_instance()->get_join();
  join_state = !join_state;
  handle_join_display(join_state);
  if (join_state)
    Lorawan_Handler::get_instance()->set_uplink_interval(static_cast<uint32_t>(lv_spinbox_get_value(ui_ValueInterval)));
  Lorawan_Handler::get_instance()->set_join(join_state);
}

void Display_Handler::swipe_event_cb() {
  if (m_calibration_active) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  if (m_display_lora_settings) {
    // up
    if (dir == 4) {
      m_display_lora_settings = false;
      Lorawan_Handler::get_instance()->set_uplink_interval(
          static_cast<uint32_t>(lv_spinbox_get_value(ui_ValueInterval)));
    }
  } else if (m_display_time_settings) {
    // down
    if (dir == 8) m_display_time_settings = false;
  } else {
    // left
    if (dir == 1) ++m_display_index;
    // right
    else if (dir == 2)
      --m_display_index;
    // up
    else if (dir == 4)
      m_display_time_settings = true;
    // down
    else if (dir == 8)
      m_display_lora_settings = true;
  }

  if (m_display_index > 6)
    m_display_index = 0;
  else if (m_display_index < 0)
    m_display_index = 6;

  // ESP_LOGI(m_tag.c_str(), "dir: %d, index: %d", dir, display_index);
  handle_screen();
}

void Display_Handler::time_save_button_pressed_cb() {
  const lv_calendar_date_t* date = lv_calendar_get_today_date(ui_Date);
  tm date_time;
  date_time.tm_year = date->year - 1900;
  date_time.tm_mon = date->month - 1;
  date_time.tm_mday = date->day;
  date_time.tm_hour = lv_arc_get_value(ui_HoursSlider);
  date_time.tm_min = lv_arc_get_value(ui_MinutesSlider);
  date_time.tm_sec = 0;
  m_current_date_time = date_time;
  timeval current_time;
  current_time.tv_sec = mktime(&m_current_date_time.value());
  settimeofday(&current_time, nullptr);
  Uart_Handler::get_instance()->send_timestamp(current_time.tv_sec);
  check_if_time_set();
}

void Display_Handler::date_change_cb() {
  lv_calendar_date_t date;
  if (lv_calendar_get_pressed_date(ui_Date, &date) != LV_RES_OK) {
    ESP_LOGE(m_tag.c_str(), "There was no date pressed!");
    return;
  }
  lv_calendar_set_today_date(ui_Date, date.year, date.month, date.day);
}

void Display_Handler::calibration_check_cb(lv_event_t* e) {
  if (!check_display_index_camera("calibration_start_cb")) return;
  if (!m_display_values.all_ir_values[m_display_index].has_value()) return;
  lv_obj_t* clicked_button = lv_event_get_target(e);
  if (!lv_obj_has_state(clicked_button, LV_STATE_CHECKED) && m_calibration_active) return;
  m_calibration_active = !m_calibration_active;
  if (m_calibration_active) lv_obj_add_state(clicked_button, LV_STATE_CHECKED);
  handle_calibration_view();
}

void Display_Handler::calibration_choice_cb(lv_event_t* e) {
  if (!m_calibration_active) return;
  lv_obj_t* clicked_button = lv_event_get_target(e);
  if (!lv_obj_has_state(clicked_button, LV_STATE_CHECKED))
    lv_obj_add_state(clicked_button, LV_STATE_CHECKED);
  else
    lv_obj_clear_state(clicked_button, LV_STATE_CHECKED);
  if (std::any_of(m_pixels.cbegin(), m_pixels.cend(),
                  [](const lv_obj_t* pixel) { return lv_obj_has_state(pixel, LV_STATE_CHECKED); }))
    return;
  m_calibration_active = false;
  handle_calibration_view();
}

void Display_Handler::select_all_cb() {
  if (std::all_of(m_pixels.cbegin(), m_pixels.cend(),
                  [](const lv_obj_t* pixel) { return lv_obj_get_state(pixel) == LV_STATE_CHECKED; }))
    for (lv_obj_t* pixel : m_pixels) lv_obj_clear_state(pixel, LV_STATE_CHECKED);
  else
    for (lv_obj_t* pixel : m_pixels) lv_obj_add_state(pixel, LV_STATE_CHECKED);
}

void Display_Handler::confirm_pixels_cb() { lv_disp_load_scr(ui_ScreenPopUp); }

void Display_Handler::confirm_calibration_cb() {
  calibration_type type = sensor;
  float calibration_temp = m_display_values.surface_temperature.value_or(-1.f);
  if (lv_dropdown_get_selected(ui_CalibOption) == 0) {
    type = manual;
    std::string value_string(lv_textarea_get_text(ui_ValueCalib));
    try {
      calibration_temp = std::stof(value_string);
    } catch (const std::invalid_argument& e) {
      ESP_LOGW(m_tag.c_str(),
               "There was an invalid calibration value entered, cancel "
               "calibration! Error: %s",
               e.what());
      return;
    }
  }

  for (size_t pixel_index = 0; pixel_index < m_pixels.size(); ++pixel_index) {
    if (!lv_obj_has_state(m_pixels[pixel_index], LV_STATE_CHECKED)) continue;
    float calibration_value = 0.f;
    if (calibration_temp > -1.f)
      calibration_value = calibration_temp - m_display_values.all_ir_values[m_display_index].value()[pixel_index];
    m_display_values.calibration_values[m_display_index][pixel_index] = calibration_value;
  }

  Uart_Handler::get_instance()->send_calibration(m_display_index, static_cast<uint8_t>(type), calibration_temp,
                                                 m_display_values.calibration_values[m_display_index]);
  m_calibration_set[m_display_index] = true;
  m_calibration_active = false;
  handle_camera_screen_load();
}

void Display_Handler::cancel_calibration_cb() {
  m_calibration_active = false;
  handle_camera_screen_load();
}

void Display_Handler::calibration_choice_cb() {
  if (lv_dropdown_get_selected(ui_CalibOption) == 0) {
    lv_obj_clear_state(ui_ValueNumpad, LV_STATE_DISABLED);
    lv_obj_clear_state(ui_ValueCalib, LV_STATE_DISABLED);
    return;
  }
  lv_obj_add_state(ui_ValueNumpad, LV_STATE_DISABLED);
  lv_obj_add_state(ui_ValueCalib, LV_STATE_DISABLED);
}
