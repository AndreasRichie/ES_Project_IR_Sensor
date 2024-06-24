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
#include "lv_port.h"
#include "lvgl.h"
#include "uart_handler.h"

#define PRINT_INTERVAL_MS 5000

#define DEBUG_VALUES 1
#define PRINT_PIXELS 0

#define DEBUG_PRINT_PRIORITY 0
#define READ_PRIORITY (configMAX_PRIORITIES - 1)
#define DISPLAY_PRIORITY (configMAX_PRIORITIES - 2)

static const char *TAG = "app_main";

uart_handler uart_handler_;
display_handler display_handler_;

int display_index = 0;

ESP_EVENT_DEFINE_BASE(VIEW_EVENT_BASE);
esp_event_loop_handle_t view_event_handle;

static void read_uart_task(void *arg) {
  while (1) {
    uart_handler_.read_from_sensor();
  }
}

#if DEBUG_VALUES
static void print_values_task(void *arg) {
  while (1) {
    uart_handler_.get_last_read_data().print_to_serial(PRINT_PIXELS);
    vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));
  }
}
#endif

static void handle_display_task(void *arg) {
  while (1) {
    display_handler_.handle_values(uart_handler_.get_last_read_data(),
                                   display_index);
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

void swipe_event_cb(lv_event_t *e) {
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  if (dir == 1)
    ++display_index;
  else if (dir == 2)
    --display_index;

  if (display_index > 6)
    display_index = 0;
  else if (display_index < 0)
    display_index = 6;

  ESP_LOGI(TAG, "dir: %d, index: %d", dir, display_index);
  display_handler_.handle_screen(display_index);
}

extern "C" void app_main(void) {
  uart_handler_.init_uart();
  ESP_ERROR_CHECK(bsp_board_init());
  display_handler_.init_display();

#if DEBUG_VALUES
  xTaskCreate(print_values_task, "print_values_task", 1024 * 6, NULL,
              DEBUG_PRINT_PRIORITY, NULL);
#endif
  xTaskCreate(read_uart_task, "read_uart_task", 1024 * 6, NULL, READ_PRIORITY,
              NULL);
  xTaskCreate(handle_display_task, "handle_display_task", 1024 * 6, NULL,
              DISPLAY_PRIORITY, NULL);

  lv_obj_add_event_cb(ui_ScreenCamera, swipe_event_cb, LV_EVENT_GESTURE, NULL);
  lv_obj_add_event_cb(ui_ScreenSensors, swipe_event_cb, LV_EVENT_GESTURE, NULL);
  // auto child_count = lv_obj_get_child_cnt(lv_scr_act());

  // ESP_LOGI(TAG, "%d", child_count);

  // auto label_screen = lv_obj_get_child(lv_scr_act(), 1);
  // ESP_LOGI(TAG, "%s", lv_label_get_text(label_screen));
  // lv_label_ins_text(label_screen, LV_LABEL_POS_LAST, "1");

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
