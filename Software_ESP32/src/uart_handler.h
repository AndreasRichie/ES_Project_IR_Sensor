#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#pragma once

#include <cobs.h>

#include <ctime>
#include <vector>

#include "driver/uart.h"
#include "measurement_data.h"

#define RXD2 20
#define TXD2 19

// receive sizes
// size necessary for IR camera data (64 pixels + pkt type + index)
#define IR_DATA_SIZE PIXEL_COUNT * sizeof(int16_t) + 2
// single floating point values size (like temperature) --> int16_t + pkt type
#define FLOAT_DATA_SIZE sizeof(int16_t) + 1
// floating point values with index size (WRT) --> int16_t + index + pkt type
#define FLOAT_IDX_DATA_SIZE sizeof(int16_t) + 2
// IR data + index + calib type + calib ref + pkt type
#define CALIB_DATA_SIZE (PIXEL_COUNT + 1) * sizeof(int16_t) + 3

// send sizes
// IR data + index + calib type + calib ref
#define CALIB_SEND_SIZE (PIXEL_COUNT + 1) * sizeof(int16_t) + 2

// biggest size necessary for transmission is calibration data (64 pixels +
// calib type + calib ref + pkt type + index + overhead + delimiter) = 135
// -> use next exp 2 value
#define RX_BUFFER 256
#define TX_BUFFER RX_BUFFER
#define DEBUG_UART 0
#define READ_TIMEOUT_MS 1000

// Type of transfer packet
#define PKT_TYPE_SENSOR_IR_CAMERA 0XB0
#define PKT_TYPE_SENSOR_SURF_TEMP 0XB1
#define PKT_TYPE_SENSOR_AIR_TEMP 0XB2
#define PKT_TYPE_SENSOR_AIR_HUM 0XB3
#define PKT_TYPE_MEAN_IR_CAMERA 0XB4
#define PKT_TYPE_MEAN_RADIANT_TEMP 0XB5
#define PKT_TYPE_CMD_SET_TIME 0xA0
#define PKT_TYPE_CMD_CALIBRATION 0XA1

#define UART_QUEUE_SIZE 10

class Uart_Handler {
 private:
  Uart_Handler() = default;
  ~Uart_Handler() = default;

 public:
  Uart_Handler(Uart_Handler& other) = delete;    // not clonable
  void operator=(const Uart_Handler&) = delete;  // not assignable
  static Uart_Handler* get_instance();

 public:
  void init_uart();
  void start_receive_task();
  void send_timestamp(const time_t& timestamp);
  void send_calibration(const uint8_t index, const uint8_t type, const float ref,
                        const pixel_values_t& calibration_values);

 private:
  static void uart_receive_task_wrapper(void* pvParameters);
  void UART_Receive_Task();

 private:
  void read_from_sensor();

  bool check_decoded_data_size(const size_t size, const size_t expected_size, const std::string& value_type);
  bool check_decoded_data_size_float(const size_t size, const std::string& value_type);
  bool check_decoded_data_size_ir_camera(const size_t size, const std::string& value_type);

  void handle_packet(const size_t size);
  void handle_pixels_received(const size_t size, const value_type& type);
  void handle_float_received(const value_type& type);
  void handle_ir_camera_mean_received(const size_t size);
  void send_data(const uint8_t& type, const std::vector<uint8_t>& data);
  void send_encoded_data(const uint8_t& type, const std::vector<uint8_t>& data);

  void send_to_display_task(const value_type& type, const uint8_t index, const std::vector<float>& values);
  void send_to_lorawan_task(const value_type& type, const uint8_t index, const float value);

 private:
  const std::string m_tag{"Uart_Handler"};
  std::array<uint8_t, RX_BUFFER> m_rx_data;
  std::array<uint8_t, RX_BUFFER> m_decoded_data;
  std::array<uint8_t, TX_BUFFER> m_encoded_data;
  std::vector<uint8_t> m_packet;
  cobs_decode_result m_cobs_dec_ret;
  QueueHandle_t m_uart_queue;
};

#endif