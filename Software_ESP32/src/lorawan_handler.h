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

class lorawan_handler {
 public:
  struct lorawan_config {
    uint32_t uplink_interval_min;
    std::array<uint8_t, 8> eui;
    std::array<uint8_t, 8> join_eui;
    std::array<uint8_t, 16> app_key;
    bool join = false;
  };

  // change to send uint16 instead of float
  enum struct value_types_lora {
    ir_camera_1,
    ir_camera_2,
    ir_camera_3,
    ir_camera_4,
    ir_camera_5,
    ir_camera_6,
    sensor_values,
    max_value_types_lora
  };

  enum struct combined_value_types {
    ir_camera_1_mean,
    ir_camera_2_mean,
    ir_camera_3_mean,
    ir_camera_4_mean,
    ir_camera_5_mean,
    ir_camera_6_mean,
    air_temperature,
    air_humidity,
    surface_temperature,
    max_combined_value_types
  };

  lorawan_handler();
  ~lorawan_handler();

  void init_lorawan();
  lorawan_config get_lorawan_config() const;
  void set_uplink_interval(const uint32_t interval);
  void set_join(const bool join);
  bool get_join() const;
  void handle_lorawan();
  void set_values(const measurement_data& to_set);

 private:
  std::array<uint8_t, 8> generate_eui();
  std::array<uint8_t, 16> generate_key();
  void set_euis_and_key();

  esp_err_t init_flash();
  bool save_config_to_flash();
  bool load_config_from_flash();

  void init_lora_mac();
  bool deinit_lora_mac();
  void process_uplink();
  void prepare_tx_frame();
  using values_vector = std::vector<uint16_t>;
  uint16_t convert_float_to_int(const float to_convert,
                                const uint8_t precision) const;
  values_vector get_values_for_type(const value_types_lora& type);
  values_vector get_pixels_for_camera(const uint8_t camera_index);
  values_vector get_values_combined();

 private:
  const std::string tag;
  SemaphoreHandle_t data_mutex;
  bool use_flash;
  uint8_t packet_count;
  std::optional<measurement_data> values;
  LmHandlerAppData_t app_data;
  bool is_init;
};

#endif