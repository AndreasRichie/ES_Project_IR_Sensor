#include "lorawan_handler.h"

#include "bootloader_random.h"
#include "esp_random.h"

lorawan_handler::lorawan_handler() : tag("lorawan_handler") {}

lorawan_handler::~lorawan_handler() {}

void lorawan_handler::init_lorawan() {
  bootloader_random_enable();
  config.eui = generate_eui();
  config.join_eui = generate_eui();
  config.app_key = generate_key();
}

lorawan_handler::lorawan_config lorawan_handler::get_lorawan_config() const {
  return config;
}

void lorawan_handler::set_uplink_interval(uint32_t interval) {
  config.uplink_interval_min = interval;
}

std::array<uint8_t, 8> lorawan_handler::generate_eui() {
  std::array<uint8_t, 8> buffer;
  esp_fill_random(static_cast<void*>(&buffer), buffer.size());
  buffer[0] = (buffer[0] & ~1) | 2;
  return buffer;
}

std::array<uint8_t, 16> lorawan_handler::generate_key() {
  std::array<uint8_t, 16> buffer;
  esp_fill_random(static_cast<void*>(&buffer), buffer.size());
  return buffer;
}