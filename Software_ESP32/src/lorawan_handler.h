#ifndef LORAWAN_HANDLER_H
#define LORAWAN_HANDLER_H

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "freertos/FreeRTOS.h"
#include "measurement_data.h"
#include "timer.h"

#define STORAGE_NAME "lorawan"
#define LOCATION_NAME "lorawan-config"

#define DEFAULT_UPLINK_INTERVAL 5

#define LORAWAN_QUEUE_SIZE 10
#define LORAWAN_TASK_TIME_MS 5

#define FIRMWARE_VERSION 0x01000000L  // 1.0.0.0

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is
 * disabled
 */
#define LORAWAN_DEFAULT_DATARATE DR_5

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE LORAMAC_HANDLER_CONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE 200

/*!
 * Number of Transmissions to send all bytes of sensor data
 */
#define MAX_COUNT_TRX NUMBER_OF_BYTES / LORAWAN_APP_DATA_BUFFER_MAX_SIZE

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only
 * for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are
 * reserved.
 */
#define LORAWAN_APP_PORT 2

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND 2000

class Lorawan_Handler {
 public:
  struct lorawan_config {
    uint32_t uplink_interval_min;
    std::array<uint8_t, 8> dev_eui;
    std::array<uint8_t, 8> join_eui;
    std::array<uint8_t, 16> app_key;
    bool join = false;
  };

  enum struct lorawan_value_types {
    ir_camera_1_mean = 0,
    ir_camera_2_mean = 1,
    ir_camera_3_mean = 2,
    ir_camera_4_mean = 3,
    ir_camera_5_mean = 4,
    ir_camera_6_mean = 5,
    mean_radiant_temp = 6,
    air_temperature = 7,
    air_humidity = 8,
    surface_temperature = 9,
    max_value_types
  };

 private:
  Lorawan_Handler() = default;
  ~Lorawan_Handler() = default;

 public:
  Lorawan_Handler(Lorawan_Handler& other) = delete;  // not clonable
  void operator=(const Lorawan_Handler&) = delete;   // not assignable
  static Lorawan_Handler* get_instance();

 public:
  void init_lorawan();
  lorawan_config get_lorawan_config() const;
  void set_uplink_interval(const uint32_t interval);
  void set_join(const bool join);
  bool get_join() const;
  void add_to_queue(const lorawan_data& data);

 private:
  static void lorawan_send_task_wrapper(void* pvParameters);
  void LoRaWAN_Send_Task();

 private:
  std::array<uint8_t, 8> generate_eui();
  std::array<uint8_t, 16> generate_key();
  void set_euis_and_key();

  esp_err_t init_flash();
  bool save_config_to_flash();
  bool load_config_from_flash();

  void handle_lorawan();
  void handle_values(const lorawan_data& values);
  void init_lora_mac();
  bool deinit_lora_mac();
  void process_uplink();
  void prepare_tx_frame();
  std::optional<int16_t> convert_optional_float_to_int(const optional_float_t& to_convert) const;
  std::optional<int16_t> get_values_for_type(const lorawan_value_types& type);

 private:
  const std::string m_tag{"Lorawan_Handler"};
  SemaphoreHandle_t m_data_mutex{NULL};
  bool m_use_flash{false};
  uint8_t m_packet_count{0};
  lorawan_values m_values;
  LmHandlerAppData_t m_app_data;
  bool m_is_init{false};
  QueueHandle_t m_lorawan_queue{NULL};
};

#endif