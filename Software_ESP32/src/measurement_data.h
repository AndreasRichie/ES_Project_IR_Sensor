#ifndef MEASUREMENT_DATA_H
#define MEASUREMENT_DATA_H

#pragma once

#include <array>
#include <optional>
#include <string>

#define PIXEL_COUNT 64
#define NUMBER_OF_CAMERAS 6

using pixel_values_t = std::array<float, PIXEL_COUNT>;
using camera_values_t = std::array<pixel_values_t, NUMBER_OF_CAMERAS>;
using optional_pixels_t = std::optional<pixel_values_t>;
using optional_pixels_camera_t = std::array<optional_pixels_t, NUMBER_OF_CAMERAS>;
using calibration_values_t = camera_values_t;
using optional_float_t = std::optional<float>;
using wall_mean_values_t = std::array<optional_float_t, NUMBER_OF_CAMERAS>;

enum struct value_type {
  pixel_values = 0,
  surface_temperature = 1,
  air_temperature = 2,
  air_humidity = 3,
  radiated_wall_temperature = 4,
  mean_radiant_temperature = 5,
  calibration = 6,
};

struct display_data {
  uint8_t type;
  uint8_t index;
  pixel_values_t data;
};

struct lorawan_data {
  uint8_t type;
  uint8_t index;
  float data;
};

struct display_values {
  optional_pixels_camera_t all_ir_values;
  std::optional<float> surface_temperature;
  float air_temperature;
  float air_humidity;
  float mean_radiant_temperature;
  calibration_values_t calibration_values;
};

struct lorawan_values {
  optional_float_t surface_temperature;
  optional_float_t air_temperature;
  optional_float_t air_humidity;
  wall_mean_values_t radiated_wall_temperatures;
  optional_float_t mean_radiant_temperature;
};

#endif