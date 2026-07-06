#include "uart_handler.h"

#include <string.h>

#include <numeric>

#include "display_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "lorawan_handler.h"
#include "priorities.h"

Uart_Handler* Uart_Handler::get_instance() {
  static Uart_Handler instance;
  return &instance;
}

void Uart_Handler::init_uart() {
  const uart_config_t uart_config = {.baud_rate = 115200,
                                     .data_bits = UART_DATA_8_BITS,
                                     .parity = UART_PARITY_DISABLE,
                                     .stop_bits = UART_STOP_BITS_1,
                                     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                     .rx_flow_ctrl_thresh = 0,
                                     .source_clk = UART_SCLK_XTAL};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUFFER, 0, UART_QUEUE_SIZE, &m_uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void Uart_Handler::start_receive_task() {
  xTaskCreate(uart_receive_task_wrapper, "UART_Receive_Task", 1024 * 4, NULL, READ_PRIORITY, NULL);
}

void Uart_Handler::send_timestamp(const time_t& timestamp) {
  ESP_LOGI(m_tag.c_str(), "Set system time of RP2040 to new value, timestamp: %lld", timestamp);
  std::array<uint8_t, sizeof(time_t)> timestamp_bytes;
  memcpy(&timestamp_bytes[0], &timestamp, timestamp_bytes.size());
  send_data(PKT_TYPE_CMD_SET_TIME, std::vector<uint8_t>(timestamp_bytes.begin(), timestamp_bytes.end()));
}

void Uart_Handler::send_calibration(const uint8_t index, const uint8_t type, const float ref,
                                    const pixel_values_t& calibration_values) {
  std::array<uint8_t, CALIB_SEND_SIZE> calib_data_bytes;
  size_t size = 0;
  calib_data_bytes[size] = index;
  ++size;
  calib_data_bytes[size] = type;
  ++size;
  const int16_t ref_int = ref * 100.f;
  memcpy(&calib_data_bytes[size], &ref_int, sizeof(int16_t));
  size += sizeof(int16_t);
  for (const auto& data_single : calibration_values) {
    const int16_t data_int = data_single * 100.f;
    memcpy(&calib_data_bytes[size], &data_int, sizeof(int16_t));
    size += sizeof(int16_t);
    if (size > CALIB_SEND_SIZE) {
      ESP_LOGE(m_tag.c_str(), "Calibration tried to send too many bytes!");
      return;
    }
  }

  send_data(PKT_TYPE_CMD_CALIBRATION, std::vector<uint8_t>(calib_data_bytes.begin(), calib_data_bytes.end()));
}

void Uart_Handler::uart_receive_task_wrapper([[maybe_unused]] void* pvParameters) {
  Uart_Handler::get_instance()->UART_Receive_Task();
}

void Uart_Handler::UART_Receive_Task() {
  uart_event_t event;
  while (1) {
    if (xQueueReceive(m_uart_queue, static_cast<void*>(&event), portMAX_DELAY) == pdTRUE) {
      if (event.type == UART_DATA) read_from_sensor();
    } else {
      ESP_LOGE(m_tag.c_str(), "xQueueReceive returned pdFALSE!");
    }
  }
}

void Uart_Handler::read_from_sensor() {
  const int rx_size = uart_read_bytes(UART_NUM_1, m_rx_data.data(), m_rx_data.size(), pdMS_TO_TICKS(READ_TIMEOUT_MS));
  if (rx_size > 0) {
#if DEBUG_UART
    std::stringstream byte_stream;
    for (int count = 0; count < rx_size; ++count)
      byte_stream << std::hex << std::setfill('0') << std::setw(2) << (int)m_rx_data[count] << " ";
    ESP_LOGI(m_tag.c_str(), "Read %d bytes: '%s'", rx_size, byte_stream.str().c_str());
#endif
    for (int index = 0; index < rx_size; ++index) {
      const auto byte = m_rx_data[index];
      if (byte != 0x00) {
        m_packet.push_back(byte);
        continue;
      }
      m_cobs_dec_ret = cobs_decode(m_decoded_data.data(), m_decoded_data.size(), m_packet.data(), m_packet.size());
      m_packet.clear();
      if (m_cobs_dec_ret.out_len == 0 || m_cobs_dec_ret.status != COBS_DECODE_OK) {
        ESP_LOGE(m_tag.c_str(), "Error while decoding COBS UART package, cobs_ret.status = %d", m_cobs_dec_ret.status);
        continue;
      }
      handle_packet(m_cobs_dec_ret.out_len);
    }
  }
}

bool Uart_Handler::check_decoded_data_size(const size_t size, const size_t expected_size,
                                           const std::string& value_type) {
  if (size != expected_size) {
    ESP_LOGE(m_tag.c_str(),
             "Bytes of data do not match expected bytes for %s! Expected: %d, "
             "received: %d",
             value_type.c_str(), expected_size, size);
    return false;
  }
  return true;
}

bool Uart_Handler::check_decoded_data_size_float(const size_t size, const std::string& value_type) {
  return check_decoded_data_size(size, FLOAT_DATA_SIZE, value_type);
}

bool Uart_Handler::check_decoded_data_size_ir_camera(const size_t size, const std::string& value_type) {
  return check_decoded_data_size(size, IR_DATA_SIZE, value_type);
}

float convert_bytes_to_float(const uint8_t* bytes) {
  int16_t data_value;
  memcpy(&data_value, bytes, sizeof(int16_t));
  return static_cast<float>(data_value) / 100.f;
}

void Uart_Handler::handle_packet(const size_t size) {
  if (size < 1) {
    return;
  }
  switch (m_decoded_data[0]) {
    case PKT_TYPE_SENSOR_IR_CAMERA:
      if (check_decoded_data_size_ir_camera(size, "ir camera data"))
        handle_pixels_received(size, value_type::pixel_values);
      break;
    case PKT_TYPE_SENSOR_SURF_TEMP:
      if (check_decoded_data_size_float(size, "surface temperature"))
        handle_float_received(value_type::surface_temperature);
      break;
    case PKT_TYPE_SENSOR_AIR_TEMP:
      if (check_decoded_data_size_float(size, "air temperature")) handle_float_received(value_type::air_temperature);
      break;
    case PKT_TYPE_SENSOR_AIR_HUM:
      if (check_decoded_data_size_float(size, "air humidity")) handle_float_received(value_type::air_humidity);
      break;
    case PKT_TYPE_MEAN_IR_CAMERA:
      if (check_decoded_data_size(size, FLOAT_IDX_DATA_SIZE, "mean IR camera")) handle_ir_camera_mean_received(size);
      break;
    case PKT_TYPE_MEAN_RADIANT_TEMP:
      if (check_decoded_data_size_float(size, "mean radiant temperature"))
        handle_float_received(value_type::mean_radiant_temperature);
      break;
    case PKT_TYPE_CMD_CALIBRATION:
      if (check_decoded_data_size(size, CALIB_DATA_SIZE, "calibration data"))
        handle_pixels_received(size, value_type::calibration);
      break;

    default:
      ESP_LOGE(m_tag.c_str(), "Invalid Paket received!");
      break;
  }
}

void Uart_Handler::handle_pixels_received(const size_t size, const value_type& type) {
  size_t size_converted = 1;

  const uint8_t camera_index = m_decoded_data[size_converted];
  if (camera_index > 5) {
    ESP_LOGE(m_tag.c_str(), "An invalid camera index was sent!");
    return;
  }
  ++size_converted;
  if (type == value_type::calibration)
    // skip calib_type and ref
    size_converted += sizeof(uint8_t) + sizeof(int16_t);
  unsigned int pixel_index = 0;
  std::vector<float> pixel_values;
  while (size_converted < size) {
    pixel_values.push_back(convert_bytes_to_float(&m_decoded_data[size_converted]));
    size_converted += sizeof(int16_t);
    ++pixel_index;
  }
  if (pixel_index != PIXEL_COUNT) {
    ESP_LOGE(m_tag.c_str(), "Not all pixels were sent!");
    return;
  }
  send_to_display_task(type, camera_index, pixel_values);
}

void Uart_Handler::handle_float_received(const value_type& type) {
  const float float_value = convert_bytes_to_float(&m_decoded_data[1]);
  send_to_display_task(type, 6, {float_value});
  send_to_lorawan_task(type, 0, {float_value});
}

void Uart_Handler::handle_ir_camera_mean_received(const size_t size) {
  const uint8_t index = m_decoded_data[1];
  if (index > 5) {
    ESP_LOGE(m_tag.c_str(), "An invalid camera index was sent!");
    return;
  }
  const float float_value = convert_bytes_to_float(&m_decoded_data[2]);
  send_to_lorawan_task(value_type::radiated_wall_temperature, index, {float_value});
}

void Uart_Handler::send_data(const uint8_t& type, const std::vector<uint8_t>& data) {
  if (data.size() > TX_BUFFER) {
    ESP_LOGE(m_tag.c_str(), "Data is too big for TX buffer!");
    return;
  }

  switch (type) {
    case PKT_TYPE_CMD_SET_TIME:
      if (!check_decoded_data_size(data.size(), sizeof(time_t), "timestamp")) return;
      break;
    case PKT_TYPE_CMD_CALIBRATION:
      if (!check_decoded_data_size(data.size(), CALIB_SEND_SIZE, "calibration")) return;
      break;
    default:
      ESP_LOGE(m_tag.c_str(), "Supposed to send invalid type!");
      return;
  }
  send_encoded_data(type, data);
}

void Uart_Handler::send_encoded_data(const uint8_t& type, const std::vector<uint8_t>& data) {
  std::vector<uint8_t> to_send;
  to_send.push_back(type);
  for (const uint8_t& byte : data) to_send.push_back(byte);

  size_t send_size = to_send.size();
  // + overhead + delimiter
  size_t encoded_size = send_size + 2;
  if (encoded_size > TX_BUFFER) {
    ESP_LOGE(m_tag.c_str(), "Tried to send to many bytes over uart, send max_size, to_send.size: %d!", send_size);
    send_size = TX_BUFFER - 2;
  }

  m_encoded_data.fill(0);
  cobs_encode_result cobs_enc_ret =
      cobs_encode(m_encoded_data.data(), m_encoded_data.size() - 1, to_send.data(), send_size);
  if (cobs_enc_ret.out_len == 0 || cobs_enc_ret.status != COBS_ENCODE_OK) {
    ESP_LOGE(m_tag.c_str(), "Error while decoding COBS UART package, cobs_ret.status = %d", cobs_enc_ret.status);
    return;
  }
  const int tx_size = uart_write_bytes(UART_NUM_1, m_encoded_data.data(), encoded_size);
  if (tx_size == -1)
    ESP_LOGE(m_tag.c_str(), "Invalid parameter for UART write!");
  else if (tx_size == 0)
    ESP_LOGE(m_tag.c_str(), "No bytes were sent via UART!");
}

void Uart_Handler::send_to_display_task(const value_type& type, const uint8_t index, const std::vector<float>& values) {
  pixel_values_t values_array;
  std::copy(values.begin(), values.end(), values_array.begin());
  Display_Handler::get_instance()->add_to_queue({static_cast<uint8_t>(type), index, values_array});
}

void Uart_Handler::send_to_lorawan_task(const value_type& type, const uint8_t index, const float value) {
  Lorawan_Handler::get_instance()->add_to_queue({static_cast<uint8_t>(type), index, value});
}
