#ifndef LORAWAN_HANDLER_H
#define LORAWAN_HANDLER_H

#pragma once

#include <array>
#include <string>

#include "freertos/FreeRTOS.h"

class lorawan_handler {
 public:
  struct lorawan_config {
    uint32_t uplink_interval_min;
    std::array<uint8_t, 8> eui;
    std::array<uint8_t, 8> join_eui;
    std::array<uint8_t, 16> app_key;
    bool auto_join;
  };

  lorawan_handler();
  ~lorawan_handler();

  void init_lorawan();
  lorawan_config get_lorawan_config() const;
  void set_uplink_interval(uint32_t interval);

 private:
  std::array<uint8_t, 8> generate_eui();
  std::array<uint8_t, 16> generate_key();

 private:
  const std::string tag;
  SemaphoreHandle_t data_mutex;
  lorawan_config config;
};

#endif