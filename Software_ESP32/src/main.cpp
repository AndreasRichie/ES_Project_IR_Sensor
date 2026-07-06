#include <math.h>
#include <ui.h>

#include <iomanip>
#include <sstream>
#include <vector>

#include "bsp_board.h"
#include "display_handler.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lorawan_handler.h"
#include "lv_port.h"
#include "lvgl.h"
#include "priorities.h"
#include "uart_handler.h"

#define PRINT_INTERVAL_MS 5000

#define DEBUG_VALUES 0
#define PRINT_PIXELS 0

// static const char *TAG = "app_main";

// Uart_Handler uart_handler_;
// lorawan_handler lorawan_handler_;
// Display_Handler display_handler_(lorawan_handler_);

ESP_EVENT_DEFINE_BASE(VIEW_EVENT_BASE);
esp_event_loop_handle_t view_event_handle;

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  ESP_LOGE("RTOS", "Stack overflow in task: %s", pcTaskName);
  abort();
}

// change UART to read at uart data event and parse packet
// send task message with value to display task and lora task
// display task handle value and change if view with value is active
// lora task handle value send if joined
// lora task use queue, new value is added to queue and run till queue is empty
// if queue is full remove oldest value
// new sensor value class necessary to know type
static void read_uart_task(void* arg) {
  while (1) {
    // uart_handler_.read_from_sensor();
    // lorawan_handler_.set_values(Uart_Handler::get_instance()->get_last_read_data());
  }
}

#if DEBUG_VALUES
static void print_values_task(void* arg) {
  while (1) {
    uart_handler_.get_last_read_data().print_to_serial(PRINT_PIXELS);
    vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));
  }
}
#endif

static void handle_display_task(void* arg) {
  while (1) {
    // display_handler_.handle_values(Uart_Handler::get_instance()->get_last_read_data());
  }
}

static void handle_lorawan_task(void* arg) {
  while (1) {
    // lorawan_handler_.handle_lorawan();
  }
}

extern "C" void app_main(void) {
  ESP_ERROR_CHECK(bsp_board_init());
  Uart_Handler::get_instance()->init_uart();
  Display_Handler::get_instance()->init_display();
  Lorawan_Handler::get_instance()->init_lorawan();

  Uart_Handler::get_instance()->start_receive_task();

#if DEBUG_VALUES
  xTaskCreate(print_values_task, "print_values_task", 1024 * 6, NULL, DEBUG_PRINT_PRIORITY, NULL);
#endif
  // xTaskCreate(read_uart_task, "read_uart_task", 1024 * 4, NULL, READ_PRIORITY, NULL);
  // xTaskCreate(handle_display_task, "handle_display_task", 1024 * 8, NULL, DISPLAY_PRIORITY, NULL);
  // xTaskCreate(handle_lorawan_task, "handle_lorawan_task", 1024 * 4, NULL, SEND_PRIORITY, NULL);
}
