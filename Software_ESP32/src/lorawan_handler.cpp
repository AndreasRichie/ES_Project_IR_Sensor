#include "lorawan_handler.h"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "Commissioning.h"
#include "LmHandlerMsgDisplay.h"
#include "RegionCommon.h"
#include "bootloader_random.h"
#include "display_handler.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "priorities.h"

static const std::string tag_gl = "Lorawan_Handler";

extern const char* MacStatusStrings[];

static std::array<uint8_t, LORAWAN_APP_DATA_BUFFER_MAX_SIZE> app_data_buffer;
static bool is_tx_frame_pending = false;
static uint32_t tx_periodicity = 0;
static TimerEvent_t tx_timer;
static Lorawan_Handler::lorawan_config config = {DEFAULT_UPLINK_INTERVAL, {0}, {0}, {0}, false};

static LmHandlerParams_t lmhandler_params = {.Region = LORAMAC_REGION_EU868,
                                             .AdrEnable = LORAWAN_ADR_STATE,
                                             .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
                                             .TxDatarate = LORAWAN_DEFAULT_DATARATE,
                                             .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
                                             .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
                                             .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
                                             .DataBuffer = app_data_buffer.data(),
                                             .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY};

Lorawan_Handler* Lorawan_Handler::get_instance() {
  static Lorawan_Handler instance;
  return &instance;
}

void Lorawan_Handler::init_lorawan() {
  m_data_mutex = xSemaphoreCreateMutex();
  if (m_data_mutex == NULL) {
    ESP_LOGE(m_tag.c_str(), "Could not create mutex!");
    ESP_ERROR_CHECK(ESP_FAIL);
  }
  m_lorawan_queue = xQueueCreate(LORAWAN_QUEUE_SIZE, sizeof(lorawan_data));
  if (m_lorawan_queue == NULL) {
    ESP_LOGE(m_tag.c_str(), "Could not create queue!");
    ESP_ERROR_CHECK(ESP_FAIL);
  }
  xTaskCreate(lorawan_send_task_wrapper, "LoRaWAN_Send_Task", 1024 * 8, NULL, SEND_PRIORITY, NULL);
  m_app_data = LmHandlerAppData_t{
      .Port = 0,
      .BufferSize = 0,
      .Buffer = app_data_buffer.data(),
  };
  xSemaphoreTake(m_data_mutex, portMAX_DELAY);
  bootloader_random_enable();
  const auto flash_return = init_flash();
  if (flash_return != ESP_OK) {
    ESP_LOGE(m_tag.c_str(), "could not init flash, error: %s!", esp_err_to_name(flash_return));
    m_use_flash = false;
    xSemaphoreGive(m_data_mutex);
    return;
  }
  if (!load_config_from_flash()) {
    set_euis_and_key();
    if (!save_config_to_flash()) ESP_LOGE(m_tag.c_str(), "could not save euis and key to flash!");
  }
  xSemaphoreGive(m_data_mutex);
}

Lorawan_Handler::lorawan_config Lorawan_Handler::get_lorawan_config() const {
  xSemaphoreTake(m_data_mutex, portMAX_DELAY);
  const auto config_copy = config;
  xSemaphoreGive(m_data_mutex);
  return config_copy;
}

void Lorawan_Handler::set_uplink_interval(const uint32_t interval) {
  xSemaphoreTake(m_data_mutex, portMAX_DELAY);
  config.uplink_interval_min = interval;
  if (m_use_flash) {
    if (!save_config_to_flash()) ESP_LOGE(m_tag.c_str(), "could not save uplink_interval to flash!");
  }
  xSemaphoreGive(m_data_mutex);
}

void Lorawan_Handler::set_join(const bool join) {
  xSemaphoreTake(m_data_mutex, portMAX_DELAY);
  config.join = join;
  if (m_use_flash) {
    if (!save_config_to_flash()) ESP_LOGE(m_tag.c_str(), "could not save join to flash!");
  }
  xSemaphoreGive(m_data_mutex);
}

bool Lorawan_Handler::get_join() const {
  xSemaphoreTake(m_data_mutex, portMAX_DELAY);
  const auto join_copy = config.join;
  xSemaphoreGive(m_data_mutex);
  return join_copy;
}

void Lorawan_Handler::add_to_queue(const lorawan_data& data) {
  if (m_lorawan_queue == NULL) {
    ESP_LOGE(m_tag.c_str(), "Queue is NULL!");
    return;
  }
  if (uxQueueSpacesAvailable(m_lorawan_queue) == 0) {
    lorawan_data data_old;
    xQueueReceive(m_lorawan_queue, &data_old, portMAX_DELAY);
  }
  xQueueSendToBack(m_lorawan_queue, &data, portMAX_DELAY);
}

void Lorawan_Handler::lorawan_send_task_wrapper([[maybe_unused]] void* pvParameters) {
  Lorawan_Handler::get_instance()->LoRaWAN_Send_Task();
}

void Lorawan_Handler::LoRaWAN_Send_Task() {
  lorawan_data data;
  while (1) {
    if (xQueueReceive(m_lorawan_queue, static_cast<void*>(&data), pdMS_TO_TICKS(LORAWAN_TASK_TIME_MS)) == pdTRUE) {
      handle_values(data);
    } else {
      handle_lorawan();
    }
  }
}

std::array<uint8_t, 8> Lorawan_Handler::generate_eui() {
  std::array<uint8_t, 8> buffer;
  esp_fill_random(static_cast<void*>(&buffer), buffer.size());
  buffer[0] = (buffer[0] & ~1) | 2;
  return buffer;
}

std::array<uint8_t, 16> Lorawan_Handler::generate_key() {
  std::array<uint8_t, 16> buffer;
  esp_fill_random(static_cast<void*>(&buffer), buffer.size());
  return buffer;
}

void Lorawan_Handler::set_euis_and_key() {
  ESP_LOGI(m_tag.c_str(), "generate new EUIs and key");
  config.dev_eui = generate_eui();
  config.join_eui = generate_eui();
  config.app_key = generate_key();
}

esp_err_t Lorawan_Handler::init_flash() {
  ESP_LOGI(m_tag.c_str(), "init flash memory");
  esp_err_t ret_value = nvs_flash_init();
  if (ret_value == ESP_ERR_NVS_NO_FREE_PAGES || ret_value == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGE(m_tag.c_str(), "error while init flash, retry after erase!");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret_value = nvs_flash_init();
  }
  return ret_value;
}

bool Lorawan_Handler::save_config_to_flash() {
  nvs_handle_t my_handle;

  esp_err_t err = nvs_open(STORAGE_NAME, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(m_tag.c_str(), "could not open flash storage, error: %s!", esp_err_to_name(err));
    return false;
  }

  err = nvs_set_blob(my_handle, LOCATION_NAME, static_cast<void*>(&config), sizeof(lorawan_config));
  if (err == ESP_OK) {
    if (nvs_commit(my_handle) != ESP_OK) ESP_LOGE(m_tag.c_str(), "could not commit change!");
  } else
    ESP_LOGE(m_tag.c_str(), "could not set blob to flash storage, error: %s!", esp_err_to_name(err));

  nvs_close(my_handle);
  return !static_cast<bool>(err);
}

bool Lorawan_Handler::load_config_from_flash() {
  nvs_handle_t my_handle;
  esp_err_t err;

  err = nvs_open(STORAGE_NAME, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(m_tag.c_str(), "could not open flash storage, error: %s!", esp_err_to_name(err));
    return false;
  }

  lorawan_config to_load;
  size_t loaded_size = sizeof(to_load);
  err = nvs_get_blob(my_handle, LOCATION_NAME, static_cast<void*>(&to_load), &loaded_size);

  if (err != ESP_OK)
    ESP_LOGE(m_tag.c_str(), "could not get blob from flash storage, error: %s!", esp_err_to_name(err));
  else if (loaded_size != sizeof(to_load))
    ESP_LOGE(m_tag.c_str(), "size of read data does not match size of config!");
  else
    config = to_load;

  nvs_close(my_handle);
  return !static_cast<bool>(err);
}

void Lorawan_Handler::handle_lorawan() {
  if (config.join) {
    if (!m_is_init) {
      init_lora_mac();
      m_is_init = true;
    }
    LmHandlerProcess();
    process_uplink();
  } else {
    if (m_is_init) {
      if (LoRaMacIsBusy()) {
        LmHandlerProcess();
        process_uplink();
      } else {
        if (deinit_lora_mac())
          m_is_init = false;
        else
          // retry deinit again in 1s
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }
}

void Lorawan_Handler::handle_values(const lorawan_data& values) {
  value_type type = static_cast<value_type>(values.type);

  switch (type) {
    case value_type::surface_temperature:
      m_values.surface_temperature = values.data;
      break;
    case value_type::air_temperature:
      m_values.air_temperature = values.data;
      break;
    case value_type::air_humidity:
      m_values.air_humidity = values.data;
      break;
    case value_type::mean_radiant_temperature:
      m_values.mean_radiant_temperature = values.data;
      break;
    case value_type::radiated_wall_temperature:
      m_values.radiated_wall_temperatures[values.index] = values.data;
      break;
    default:
      ESP_LOGE(m_tag.c_str(), "An unhandled value_type occured!");
      break;
  }
}

static void on_mac_mcps_request(LoRaMacStatus_t status, McpsReq_t* mcps_request, TimerTime_t next_tx) {
  switch (mcps_request->Type) {
    case MCPS_CONFIRMED: {
      ESP_LOGI(tag_gl.c_str(), "Send confirm Packet, status: %s", MacStatusStrings[status]);
      break;
    }
    case MCPS_UNCONFIRMED: {
      ESP_LOGI(tag_gl.c_str(), "Send unconfirm Packet, status: %s", MacStatusStrings[status]);
      break;
    }
    default: {
      break;
    }
  }
  if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
    ESP_LOGI(tag_gl.c_str(), "Next Tx in  : %lu [ms]", next_tx);
  }
}

static void on_mac_mlme_request(LoRaMacStatus_t status, MlmeReq_t* mlme_req, TimerTime_t next_tx) {
  switch (mlme_req->Type) {
    case MLME_JOIN: {
      ESP_LOGI(tag_gl.c_str(), "Start join, status: %s", MacStatusStrings[status]);
      break;
    }
    case MLME_LINK_CHECK: {
      ESP_LOGI(tag_gl.c_str(), "Link check, status: %s", MacStatusStrings[status]);
      break;
    }
    case MLME_DEVICE_TIME: {
      ESP_LOGI(tag_gl.c_str(), "Device time, status: %s", MacStatusStrings[status]);
      break;
    }
    case MLME_TXCW: {
      ESP_LOGI(tag_gl.c_str(), "Txcw, status: %s", MacStatusStrings[status]);
      break;
    }
    default: {
      break;
    }
  }
  if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
    ESP_LOGI(tag_gl.c_str(), "Next Tx in  : %lu [ms]", next_tx);
  }
}

static void update_timer_periodicity() {
  TimerStop(&tx_timer);
  TimerSetValue(&tx_timer, tx_periodicity);
  TimerStart(&tx_timer);
}

static void on_tx_timer_event(void* context) {
  is_tx_frame_pending = true;
  update_timer_periodicity();
}

static void on_join_request(LmHandlerJoinParams_t* params) {
  if (params->CommissioningParams->IsOtaaActivation == true) {
    if (params->Status == LORAMAC_HANDLER_SUCCESS) {
      ESP_LOGI(tag_gl.c_str(), "Join Successful , DevAddr: %08lX", params->CommissioningParams->DevAddr);
      Display_Handler::get_instance()->set_joined_icon(true);
    } else {
      ESP_LOGE(tag_gl.c_str(), "Join fail!");
      Display_Handler::get_instance()->set_joined_icon(false);
    }
  } else {
    ESP_LOGI(tag_gl.c_str(), "Joined, DevAddr: %08lX", params->CommissioningParams->DevAddr);
    Display_Handler::get_instance()->set_joined_icon(true);
  }

  if (params->Status == LORAMAC_HANDLER_ERROR) {
    if (config.join) LmHandlerJoin();
  } else {
    if (LmHandlerRequestClass(CLASS_C) != LORAMAC_HANDLER_SUCCESS) ESP_LOGE(tag_gl.c_str(), "Request class fail!");

    on_tx_timer_event(nullptr);  // send data right now
  }
}

static std::string c_array_to_string(const uint8_t* array, const uint8_t size) {
  std::vector<uint8_t> data(array, array + size);
  std::stringstream string_result;
  string_result << std::hex << std::setfill('0');
  for (const auto& element : data) {
    string_result << std::setw(2) << static_cast<unsigned int>(element);
  }
  return string_result.str();
}

static void on_tx_data(LmHandlerTxParams_t* params) {
  MibRequestConfirm_t mibGet;

  std::string ack_text = "unconfirmed";

  if (params->AppData.BufferSize != 0 && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET)) {
    if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG) {
      if (params->AckReceived == 0) ESP_LOGW(tag_gl.c_str(), "NACK received!");
      ack_text = (params->AckReceived != 0) ? "confirm-ACK" : "confirm-NACK";
    }

    mibGet.Type = MIB_CHANNELS;
    LoRaMacMibGetRequestConfirm(&mibGet);
    ESP_LOGI(tag_gl.c_str(), "--> Uplink frame(%lu), FREQ:%lu, DR:%d, %s", params->UplinkCounter,
             mibGet.Param.ChannelList[params->Channel].Frequency, params->Datarate, ack_text.c_str());
    const auto data_string = c_array_to_string(params->AppData.Buffer, params->AppData.BufferSize);
    ESP_LOGI(tag_gl.c_str(), "Port:%d, data: %s", params->AppData.Port, data_string.c_str());
  }
}

static LmHandlerCallbacks_t lmhandler_callbacks = {.GetBatteryLevel = nullptr,
                                                   .GetTemperature = nullptr,
                                                   .GetRandomSeed = nullptr,
                                                   .OnMacProcess = nullptr,
                                                   .OnNvmDataChange = nullptr,
                                                   // maybe add empty function
                                                   .OnNetworkParametersChange = nullptr,
                                                   .OnMacMcpsRequest = on_mac_mcps_request,
                                                   .OnMacMlmeRequest = on_mac_mlme_request,
                                                   .OnJoinRequest = on_join_request,
                                                   .OnTxData = on_tx_data,
                                                   .OnRxData = nullptr,
                                                   // maybe add empty function
                                                   .OnClassChange = nullptr,
                                                   // maybe add empty function
                                                   .OnBeaconStatusChange = nullptr,
                                                   .OnSysTimeUpdate = nullptr};

static void calculate_default_periodicity() {
  tx_periodicity = config.uplink_interval_min * 60 * 1000 + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
}

static void on_tx_periodicity_changed(uint32_t periodicity) {
  tx_periodicity = periodicity;

  // Revert to application default periodicity
  if (tx_periodicity == 0) calculate_default_periodicity();

  update_timer_periodicity();
}

static void on_tx_frame_ctrl_changed(LmHandlerMsgTypes_t is_tx_confirmed) {
  lmhandler_params.IsTxConfirmed = is_tx_confirmed;
}
static void on_ping_slot_periodicity_changed(uint8_t ping_slot_periodicity) {
  lmhandler_params.PingSlotPeriodicity = ping_slot_periodicity;
}

static LmhpComplianceParams_t lmhpcomplience_params = {
    .FwVersion{FIRMWARE_VERSION},
    .OnTxPeriodicityChanged = on_tx_periodicity_changed,
    .OnTxFrameCtrlChanged = on_tx_frame_ctrl_changed,
    .OnPingSlotPeriodicityChanged = on_ping_slot_periodicity_changed};

void Lorawan_Handler::init_lora_mac() {
  ESP_LOGI(m_tag.c_str(), "Init LoRaMAC");
  if (LmHandlerInit(&lmhandler_callbacks, &lmhandler_params) != LORAMAC_HANDLER_SUCCESS) {
    // Fatal error, endless loop.
    while (1) {
      ESP_LOGE(m_tag.c_str(), "LoRaMac wasn't properly initialized!");
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }

  MibRequestConfirm_t mib_req;

  // dev_eui
  mib_req.Type = MIB_DEV_EUI;
  mib_req.Param.DevEui = config.dev_eui.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(m_tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  // join eui
  mib_req.Type = MIB_JOIN_EUI;
  mib_req.Param.JoinEui = config.join_eui.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(m_tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  // app key
  mib_req.Type = MIB_APP_KEY;
  mib_req.Param.AppKey = config.app_key.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(m_tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  mib_req.Type = MIB_NWK_KEY;
  mib_req.Param.NwkKey = config.app_key.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(m_tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  LmHandlerSetSystemMaxRxError(50);
  LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &lmhpcomplience_params);

  LmHandlerJoin1(true);

  // TX timer
  calculate_default_periodicity();
  TimerInit(&tx_timer, on_tx_timer_event);
  TimerSetValue(&tx_timer, tx_periodicity);
}

bool Lorawan_Handler::deinit_lora_mac() {
  ESP_LOGI(m_tag.c_str(), "Deinit LoRaMAC");
  is_tx_frame_pending = false;
  TimerStop(&tx_timer);
  LoRaMacStop();
  LoRaMacStatus_t status = LoRaMacDeInitialization();
  if (status != LORAMAC_STATUS_OK) {
    ESP_LOGE(m_tag.c_str(), "LoRaMacDeInitialization error: %d", status);
    return false;
  }
  Display_Handler::get_instance()->set_joined_icon(false);
  return true;
}

void Lorawan_Handler::process_uplink() {
  if (is_tx_frame_pending == true) prepare_tx_frame();
}

void Lorawan_Handler::prepare_tx_frame() {
  if (LmHandlerIsBusy() == true) return;

  if (m_packet_count >= static_cast<uint8_t>(lorawan_value_types::max_value_types)) {
    m_packet_count = 0;
    is_tx_frame_pending = false;
    return;
  }

  u_int8_t index = 0;

  xSemaphoreTake(m_data_mutex, portMAX_DELAY);

  const auto value_to_send = get_values_for_type(static_cast<lorawan_value_types>(m_packet_count));

  if (!value_to_send.has_value()) {
    ESP_LOGE(m_tag.c_str(), "No value to send for type %d!", m_packet_count);
    ++m_packet_count;
    xSemaphoreGive(m_data_mutex);
    return;
  }

  m_app_data.Buffer[index] = m_packet_count;
  ++index;

  m_app_data.Buffer[index++] = ((value_to_send.value()) >> 8) & 0xff;
  m_app_data.Buffer[index++] = (value_to_send.value()) & 0xff;

  xSemaphoreGive(m_data_mutex);

  m_app_data.BufferSize = index;
  m_app_data.Port = LORAWAN_APP_PORT;

  if (LmHandlerSend(&m_app_data, lmhandler_params.IsTxConfirmed) == LORAMAC_HANDLER_SUCCESS) {
    ESP_LOGI(m_tag.c_str(), "packet with type %d successfully sent!", m_packet_count);
    ++m_packet_count;
  }
}

std::optional<int16_t> Lorawan_Handler::convert_optional_float_to_int(const optional_float_t& to_convert) const {
  if (!to_convert.has_value()) return {};
  return static_cast<int16_t>(10.f * to_convert.value());
}

std::optional<int16_t> Lorawan_Handler::get_values_for_type(const lorawan_value_types& type) {
  const uint8_t type_as_int = static_cast<uint8_t>(type);
  ESP_LOGI(m_tag.c_str(), "get values for packet with type: %d", type_as_int);

  switch (type) {
    case lorawan_value_types::ir_camera_1_mean:
    case lorawan_value_types::ir_camera_2_mean:
    case lorawan_value_types::ir_camera_3_mean:
    case lorawan_value_types::ir_camera_4_mean:
    case lorawan_value_types::ir_camera_5_mean:
    case lorawan_value_types::ir_camera_6_mean:
      return convert_optional_float_to_int(m_values.radiated_wall_temperatures[type_as_int]);
    case lorawan_value_types::mean_radiant_temp:
      return convert_optional_float_to_int(m_values.mean_radiant_temperature);
    case lorawan_value_types::air_temperature:
      return convert_optional_float_to_int(m_values.air_temperature);
    case lorawan_value_types::air_humidity:
      return convert_optional_float_to_int(m_values.air_humidity);
    case lorawan_value_types::surface_temperature:
      return convert_optional_float_to_int(m_values.surface_temperature);

    default:
      ESP_LOGE(m_tag.c_str(), "Undefined type!");
      return {};
  }
}
