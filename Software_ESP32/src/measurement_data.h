#ifndef MEASUREMENT_DATA_H
#define MEASUREMENT_DATA_H

#pragma once

#include <array>
#include <string>

#define PIXEL_COUNT 64
#define NUMBER_OF_CAMERAS 6

struct measurement_data {
  using pixel_values = std::array<float, PIXEL_COUNT>;
  std::array<pixel_values, NUMBER_OF_CAMERAS> ir_camera_data;
  std::array<float, NUMBER_OF_CAMERAS> ir_camera_means;
  float air_temp;
  float air_rH;
  float surface_temp;

  const std::string tag = "measurement_data";
  void print_to_serial(const bool &pixels);
};

#endif