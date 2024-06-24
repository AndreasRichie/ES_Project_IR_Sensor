#include "uart_handler.h"

#include <string.h>

#include <numeric>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

uart_handler::uart_handler() : tag("read_uart_task") {}

uart_handler::~uart_handler() = default;

void uart_handler::init_uart() {
  const uart_config_t uart_config = {.baud_rate = 115200,
                                     .data_bits = UART_DATA_8_BITS,
                                     .parity = UART_PARITY_DISABLE,
                                     .stop_bits = UART_STOP_BITS_1,
                                     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                     .source_clk = UART_SCLK_XTAL};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUFFER, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD2, RXD2, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
}

void uart_handler::read_from_sensor() {
  const int rx_size =
      uart_read_bytes(UART_NUM_1, rx_data.data(), rx_data.size(),
                      pdMS_TO_TICKS(READ_INTERVAL_MS));
  if (rx_size > 0) {
#if DEBUG_UART
    std::stringstream byte_stream;
    for (int count = 0; count < rx_size; ++count)
      byte_stream << std::hex << std::setfill('0') << std::setw(2)
                  << (int)rx_data[count] << " ";
    ESP_LOGI(tag.c_str(), "Read %d bytes: '%s'", rx_size,
             byte_stream.str().c_str());
#endif
    for (int index = 0; index < rx_size; ++index) {
      const auto byte = rx_data[index];
      if (byte != 0x00) {
        packet.push_back(byte);
        continue;
      }
      cobs_ret = cobs_decode(decoded_data.data(), decoded_data.size(),
                             packet.data(), packet.size());
      packet.clear();
      if (cobs_ret.out_len == 0 || cobs_ret.status != COBS_DECODE_OK) {
        ESP_LOGE(tag.c_str(),
                 "Error while decoding COBS UART package, cobs_ret.status = %d",
                 cobs_ret.status);
        continue;
      }
      handle_packet(cobs_ret.out_len);
    }
  }
}

measurement_data uart_handler::get_last_read_data() const { return read_data; }

void uart_handler::handle_packet(const size_t size) {
  if (size < 1) {
    return;
  }
  switch (decoded_data[0]) {
    case PKT_TYPE_SENSOR_IR_CAMERA_1: {
      handle_camera_received(0, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_2: {
      handle_camera_received(1, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_3: {
      handle_camera_received(2, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_4: {
      handle_camera_received(3, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_5: {
      handle_camera_received(4, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_6: {
      handle_camera_received(5, size);
      break;
    }
    case PKT_TYPE_SENSOR_SURF_TEMP: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &decoded_data[1], sizeof(float));
      read_data.surface_temp = dataval;
      break;
    }
    case PKT_TYPE_SENSOR_AIR_TEMP: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &decoded_data[1], sizeof(float));
      read_data.air_temp = dataval;
      break;
    }
    case PKT_TYPE_SENSOR_AIR_HUM: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &decoded_data[1], sizeof(float));
      read_data.air_rH = dataval;
      break;
    }
    default:
      ESP_LOGE(tag.c_str(), "Invalid Paket received!");
      break;
  }
}

void uart_handler::handle_camera_received(const int camera_index,
                                          const size_t size) {
  size_t size_converted = 1;
  unsigned int pixel_index = 0;
  while (size_converted < size) {
    float dataval;
    memcpy(&dataval, &decoded_data[size_converted], sizeof(float));
    read_data.ir_camera_data[camera_index][pixel_index] = dataval;
    size_converted += sizeof(float);
    ++pixel_index;
  }
  if (pixel_index != PIXEL_COUNT) {
    ESP_LOGE(tag.c_str(), "Not all pixels were sent!");
    return;
  }
  read_data.ir_camera_means[camera_index] =
      std::accumulate(read_data.ir_camera_data[camera_index].cbegin(),
                      read_data.ir_camera_data[camera_index].cend(), 0.0) /
      PIXEL_COUNT;
}
