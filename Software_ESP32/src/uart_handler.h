#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#pragma once

#include <cobs.h>

#include <vector>

#include "measurement_data.h"

#define RXD2 20
#define TXD2 19
#define RX_BUFFER 512
#define DEBUG_UART 0
#define READ_INTERVAL_MS 10

// Type of transfer packet
#define PKT_TYPE_SENSOR_IR_CAMERA_1 0XB0
#define PKT_TYPE_SENSOR_IR_CAMERA_2 0XB1
#define PKT_TYPE_SENSOR_IR_CAMERA_3 0XB2
#define PKT_TYPE_SENSOR_IR_CAMERA_4 0XB3
#define PKT_TYPE_SENSOR_IR_CAMERA_5 0XB4
#define PKT_TYPE_SENSOR_IR_CAMERA_6 0XB5
#define PKT_TYPE_SENSOR_SURF_TEMP 0XB6
#define PKT_TYPE_SENSOR_AIR_TEMP 0XB7
#define PKT_TYPE_SENSOR_AIR_HUM 0XB8

class uart_handler {
 public:
  uart_handler();
  ~uart_handler();
  void init_uart();
  void read_from_sensor();
  measurement_data get_last_read_data() const;

 private:
  void handle_packet(const size_t size);
  void handle_camera_received(const int camera_index, const size_t size);

 private:
  measurement_data read_data;
  const std::string tag;
  std::array<uint8_t, RX_BUFFER> rx_data;
  std::array<uint8_t, RX_BUFFER> decoded_data;
  std::vector<uint8_t> packet;
  cobs_decode_result cobs_ret;
};

#endif