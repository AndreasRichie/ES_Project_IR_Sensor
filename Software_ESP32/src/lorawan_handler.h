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

#define FIRMWARE_VERSION 0x01000000  // 1.0.0.0

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

// #define IR_CAMERA_1_PART_1 0
// #define IR_CAMERA_1_PART_2 1
// #define IR_CAMERA_2_PART_1 2
// #define IR_CAMERA_2_PART_2 3
// #define IR_CAMERA_3_PART_1 4
// #define IR_CAMERA_3_PART_2 5
// #define IR_CAMERA_4_PART_1 6
// #define IR_CAMERA_4_PART_2 7
// #define IR_CAMERA_5_PART_1 8
// #define IR_CAMERA_5_PART_2 9
// #define IR_CAMERA_6_PART_1 10
// #define IR_CAMERA_6_PART_2 11
// #define SENSOR_VALUES 12
// #define VALUES_MAX_INDEX 13

// #define IR_CAMERA_1_MEAN 0
// #define IR_CAMERA_2_MEAN 1
// #define IR_CAMERA_3_MEAN 2
// #define IR_CAMERA_4_MEAN 3
// #define IR_CAMERA_5_MEAN 4
// #define IR_CAMERA_6_MEAN 5
// #define SENSOR_AIR_TEMP 6
// #define SENSOR_AIR_HUM 7
// #define SENSOR_SURF_TEMP 8

class lorawan_handler {
 public:
  struct lorawan_config {
    uint32_t uplink_interval_min;
    std::array<uint8_t, 8> eui;
    std::array<uint8_t, 8> join_eui;
    std::array<uint8_t, 16> app_key;
    bool join = false;
  };

  enum struct value_types_lora {
    ir_camera_1_part_1,
    ir_camera_1_part_2,
    ir_camera_2_part_1,
    ir_camera_2_part_2,
    ir_camera_3_part_1,
    ir_camera_3_part_2,
    ir_camera_4_part_1,
    ir_camera_4_part_2,
    ir_camera_5_part_1,
    ir_camera_5_part_2,
    ir_camera_6_part_1,
    ir_camera_6_part_2,
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
  using values_vector = std::vector<float>;
  values_vector get_values_for_type(const value_types_lora& type);
  values_vector get_pixels_for_camera(const uint8_t camera_index,
                                      const bool second_half);
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