#include <math.h>
#include <ui.h>

#include <iomanip>
#include <numeric>
#include <sstream>
#include <vector>

#include "bsp_board.h"
#include "cobs.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lv_port.h"
#include "lvgl.h"
// #include "lvgl/examples/lv_examples.h"
#include "measurement_data.h"
// #include "model/indicator_model.h"
// #include "view/indicator_view.h"

// #include "indicator_controller.h"

#define RXD2 20
#define TXD2 19
#define RX_BUFFER 512

#define READ_INTERVAL_MS 10
#define PRINT_INTERVAL_MS 5000
#define DISPLAY_INTERVAL_MS 100

#define DEBUG_UART 0
#define DEBUG_VALUES 1
#define PRINT_PIXELS 0

#define DEBUG_PRINT_PRIORITY 0
#define READ_PRIORITY (configMAX_PRIORITIES - 1)
#define DISPLAY_PRIORITY (configMAX_PRIORITIES - 2)

#define MINTEMP 20
#define MAXTEMP 35

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

// the colors we will be using
const std::vector<uint16_t> camColors = {
    0x480F, 0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810,
    0x3010, 0x3010, 0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010,
    0x2010, 0x1810, 0x1810, 0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811,
    0x0811, 0x0811, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0031, 0x0031,
    0x0051, 0x0072, 0x0072, 0x0092, 0x00B2, 0x00B2, 0x00D2, 0x00F2, 0x00F2,
    0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192, 0x0192, 0x01B2, 0x01D2,
    0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273, 0x0293, 0x02B3,
    0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373, 0x0394,
    0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
    0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554,
    0x0574, 0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571,
    0x0591, 0x0591, 0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE,
    0x05AE, 0x05AD, 0x05AD, 0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB,
    0x05CA, 0x05CA, 0x05CA, 0x05C9, 0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7,
    0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5, 0x05E5, 0x0604, 0x0604, 0x0604,
    0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621, 0x0621, 0x0620, 0x0620,
    0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640, 0x1E40, 0x1E40,
    0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60, 0x3E60,
    0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
    0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0,
    0x8EA0, 0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0,
    0xB6E0, 0xB6E0, 0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0,
    0xD700, 0xDF00, 0xDEE0, 0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640,
    0xE620, 0xE600, 0xE5E0, 0xE5C0, 0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520,
    0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480, 0xE460, 0xEC40, 0xEC20, 0xEC00,
    0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40, 0xEB20, 0xEB00, 0xEAE0,
    0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200, 0xF1E0, 0xF1C0,
    0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0, 0xF080,
    0xF060, 0xF040, 0xF020, 0xF800,
};

static const char *TAG = "app_main";

measurement_data data_read;

ESP_EVENT_DEFINE_BASE(VIEW_EVENT_BASE);
esp_event_loop_handle_t view_event_handle;

void init_uart(void) {
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

void handle_camera_received(const uint8_t *buffer, const int camera_index,
                            const size_t &size, const std::string &tag) {
  size_t size_converted = 1;
  unsigned int pixel_index = 0;
  while (size_converted < size) {
    float dataval;
    memcpy(&dataval, &buffer[size_converted], sizeof(float));
    data_read.ir_camera_data[camera_index][pixel_index] = dataval;
    size_converted += sizeof(float);
    ++pixel_index;
  }
  if (pixel_index != PIXEL_COUNT) {
    ESP_LOGE(tag.c_str(), "Not all pixels were sent!");
    return;
  }
  data_read.ir_camera_means[camera_index] =
      std::accumulate(data_read.ir_camera_data[camera_index].cbegin(),
                      data_read.ir_camera_data[camera_index].cend(), 0.0) /
      PIXEL_COUNT;
}

void handle_packet(const uint8_t *buffer, size_t size, const std::string &tag) {
  if (size < 1) {
    return;
  }
  switch (buffer[0]) {
    case PKT_TYPE_SENSOR_IR_CAMERA_1: {
      handle_camera_received(buffer, 0, size, tag);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_2: {
      handle_camera_received(buffer, 1, size, tag);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_3: {
      handle_camera_received(buffer, 2, size, tag);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_4: {
      handle_camera_received(buffer, 3, size, tag);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_5: {
      handle_camera_received(buffer, 4, size, tag);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_6: {
      handle_camera_received(buffer, 5, size, tag);
      break;
    }
    case PKT_TYPE_SENSOR_SURF_TEMP: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &buffer[1], sizeof(float));
      data_read.surface_temp = dataval;
      break;
    }
    case PKT_TYPE_SENSOR_AIR_TEMP: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &buffer[1], sizeof(float));
      data_read.air_temp = dataval;
      break;
    }
    case PKT_TYPE_SENSOR_AIR_HUM: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &buffer[1], sizeof(float));
      data_read.air_rH = dataval;
      break;
    }
    default:
      ESP_LOGE(tag.c_str(), "Invalid Paket received!");
      break;
  }
}

static void read_uart_task(void *arg) {
  static const std::string read_uart_task_tag = "read_uart_task";
  std::array<uint8_t, RX_BUFFER> rx_data;
  std::array<uint8_t, RX_BUFFER> decoded_data;
  std::vector<uint8_t> packet;
  cobs_decode_result cobs_ret;
  while (1) {
    const int rx_size =
        uart_read_bytes(UART_NUM_1, rx_data.data(), rx_data.size(),
                        pdMS_TO_TICKS(READ_INTERVAL_MS));
    if (rx_size > 0) {
#if DEBUG_UART
      std::stringstream byte_stream;
      for (int count = 0; count < rx_size; ++count)
        byte_stream << std::hex << std::setfill('0') << std::setw(2)
                    << (int)rx_data[count] << " ";
      ESP_LOGI(read_uart_task_tag.c_str(), "Read %d bytes: '%s'", rx_size,
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
          ESP_LOGE(
              read_uart_task_tag.c_str(),
              "Error while decoding COBS UART package, cobs_ret.status = %d",
              cobs_ret.status);
          continue;
        }
        handle_packet(decoded_data.data(), cobs_ret.out_len,
                      read_uart_task_tag);
      }
    }
  }
}

#if DEBUG_VALUES
static void print_values_task(void *arg) {
  while (1) {
    data_read.print_to_serial(PRINT_PIXELS);
    vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));
  }
}
#endif

static void handle_display_task(void *arg) {
  static const std::string handle_display_task_tag = "handle_display_task";

  while (1) {
    auto grid = lv_obj_get_child(lv_scr_act(), 0);

    float colorTemp;

    if (grid == NULL)
      ESP_LOGE(handle_display_task_tag.c_str(), "Nullptr for grid!");
    else
      for (unsigned int index = 0; index < PIXEL_COUNT; ++index) {
        const auto pixel_object = lv_obj_get_child(grid, index);
        lv_obj_t *button_label;
        if (lv_obj_get_child_cnt(pixel_object) == 0)
          button_label = lv_label_create(pixel_object);
        else
          button_label = lv_obj_get_child(pixel_object, 0);
        const auto pixel_value = data_read.ir_camera_data[0][index];
        lv_label_set_text_fmt(button_label, "%.2f", pixel_value);
        lv_obj_center(button_label);

        if (pixel_value >= MAXTEMP)
          colorTemp = MAXTEMP;
        else if (pixel_value <= MINTEMP)
          colorTemp = MINTEMP;
        else
          colorTemp = pixel_value;

        uint8_t colorIndex = std::round(
            camColors.size() / (MAXTEMP - MINTEMP) * colorTemp -
            camColors.size() / ((float)MAXTEMP / (float)MINTEMP - 1));
        lv_color16_t color;
        color.full = camColors[colorIndex];
        lv_obj_set_style_bg_color(pixel_object, color, LV_PART_MAIN);
      }
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
  }
}

static void btn_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *btn = lv_event_get_target(e);
  if (code == LV_EVENT_CLICKED) {
    static uint8_t cnt = 0;
    cnt++;

    /*Get the first child of the button which is the label and change its text*/
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    lv_label_set_text_fmt(label, "Button: %d", cnt);
  }
}

extern "C" void app_main(void) {
  init_uart();
  ESP_ERROR_CHECK(bsp_board_init());

  lv_port_init();
  ui_init();

#if DEBUG_VALUES
  xTaskCreate(print_values_task, "print_values_task", 1024 * 3, NULL,
              DEBUG_PRINT_PRIORITY, NULL);
#endif
  xTaskCreate(read_uart_task, "read_uart_task", 1024 * 4, NULL, READ_PRIORITY,
              NULL);
  xTaskCreate(handle_display_task, "handle_display_task", 1024 * 4, NULL,
              DISPLAY_PRIORITY, NULL);

  auto child_count = lv_obj_get_child_cnt(lv_scr_act());

  ESP_LOGI(TAG, "%d", child_count);

  auto label_screen = lv_obj_get_child(lv_scr_act(), 1);
  ESP_LOGI(TAG, "%s", lv_label_get_text(label_screen));
  lv_label_ins_text(label_screen, LV_LABEL_POS_LAST, "1");

  // lv_obj_t *btn =
  //     lv_btn_create(lv_scr_act()); /*Add a button the current screen*/
  // lv_obj_set_pos(btn, 10, 10);     /*Set its position*/
  // lv_obj_set_size(btn, 120, 50);   /*Set its size*/
  // lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL,
  //                     NULL); /*Assign a callback to the button*/

  // lv_obj_t *label = lv_label_create(btn); /*Add a label to the button*/
  // lv_label_set_text(label, "Button");     /*Set the labels text*/
  // lv_obj_center(label);

  // lv_port_init();

  //   esp_event_loop_args_t view_event_task_args = {
  //       .queue_size = 10,
  //       .task_name = "view_event_task",
  //       .task_priority = uxTaskPriorityGet(NULL),
  //       .task_stack_size = 10240,
  //       .task_core_id = tskNO_AFFINITY};

  //   ESP_ERROR_CHECK(
  //       esp_event_loop_create(&view_event_task_args, &view_event_handle));

  //   lv_port_sem_take();
  //   indicator_view_init();
  //   lv_port_sem_give();

  // indicator_model_init();
  // indicator_controller_init();

  //   static char buffer[128]; /* Make sure buffer is enough for `sprintf` */
  //   while (1) {
  //     // sprintf(buffer, "   Biggest /     Free /    Total\n"
  //     //         "\t  DRAM : [%8d / %8d / %8d]\n"
  //     //         "\t PSRAM : [%8d / %8d / %8d]",
  //     //         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
  //     //         heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
  //     //         heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
  //     //         heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
  //     //         heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
  //     //         heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
  //     // ESP_LOGI("MEM", "%s", buffer);

  //     vTaskDelay(pdMS_TO_TICKS(10000));
  //   }
}
