#include "measurement_data.h"

#include <sstream>

#include "esp_log.h"

void measurement_data::print_to_serial(const bool &pixels) {
  std::stringstream stream;
  stream.precision(4);
  stream << "Measurement results:\n";
  for (unsigned int camera_index = 0; camera_index < NUMBER_OF_CAMERAS;
       ++camera_index) {
    stream << "\tIR Camera " << camera_index + 1 << ":\r\n";
    if (pixels) {
      stream << "\t\tPixel Values: ";
      for (auto &pixel : ir_camera_data[camera_index]) stream << pixel << ", ";
      stream << "\n";
    }
    stream << "\t\tMean Value: " << ir_camera_means[camera_index] << "\n";
  }
  stream << "\tAir Temperature: " << air_temp << "\n";
  stream << "\tAir Humidity: " << air_rH << "\n";
  stream << "\tSurface Temperature: " << surface_temp;
  ESP_LOGI(tag.c_str(), "%s", stream.str().c_str());
}
