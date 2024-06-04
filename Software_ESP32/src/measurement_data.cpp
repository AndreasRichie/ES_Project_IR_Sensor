#include "measurement_data.h"

#include <Arduino.h>

void measurement_data::print_to_serial(const bool &pixels) {
  Serial.println("Measurement results:");
  for (unsigned int camera_index = 0; camera_index < NUMBER_OF_CAMERAS;
       ++camera_index) {
    Serial.printf("\tIR Camera %d:\r\n", camera_index + 1);
    if (pixels) {
      Serial.print("\t\tPixel Values: ");
      for (auto &pixel : ir_camera_data[camera_index])
        Serial.printf("%.2f, ", pixel);
      Serial.println("");
    }
    Serial.print("\t\tMean Value: ");
    Serial.println(ir_camera_means[camera_index]);
  }
  Serial.print("\tAir Temperature: ");
  Serial.println(air_temp);
  Serial.print("\tAir Humidity: ");
  Serial.println(air_rH);
  Serial.print("\tSurface Temperature: ");
  Serial.println(surface_temp);
}
