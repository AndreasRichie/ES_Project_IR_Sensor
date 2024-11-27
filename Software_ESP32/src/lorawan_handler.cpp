#include "lorawan_handler.h"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "Commissioning.h"
#include "LmHandlerMsgDisplay.h"
#include "RegionCommon.h"
#include "bootloader_random.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"

static const std::string tag_gl = "lorawan_handler";

extern const char *MacStatusStrings[];

static std::array<uint8_t, LORAWAN_APP_DATA_BUFFER_MAX_SIZE> app_data_buffer;
static bool is_tx_frame_pending = false;
static uint32_t tx_periodicity = 0;
static TimerEvent_t tx_timer;
static lorawan_handler::lorawan_config config = {
    DEFAULT_UPLINK_INTERVAL, {0}, {0}, {0}, false};
static bool values_set = false;

static LmHandlerParams_t lmhandler_params = {
    .Region = LORAMAC_REGION_EU868,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = app_data_buffer.data(),
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY};

lorawan_handler::lorawan_handler()
    : tag("lorawan_handler"),
      data_mutex(xSemaphoreCreateMutex()),
      use_flash(true),
      packet_count(0),
      app_data({
          .Port = 0,
          .BufferSize = 0,
          .Buffer = app_data_buffer.data(),
      }),
      is_init(false) {}

lorawan_handler::~lorawan_handler() = default;

void lorawan_handler::init_lorawan() {
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  bootloader_random_enable();
  const auto flash_return = init_flash();
  if (flash_return != ESP_OK) {
    ESP_LOGE(tag.c_str(), "could not init flash, error: %s!",
             esp_err_to_name(flash_return));
    use_flash = false;
    xSemaphoreGive(data_mutex);
    return;
  }
  if (!load_config_from_flash()) {
    set_euis_and_key();
    if (!save_config_to_flash())
      ESP_LOGE(tag.c_str(), "could not save to flash!");
  }
  xSemaphoreGive(data_mutex);
}

lorawan_handler::lorawan_config lorawan_handler::get_lorawan_config() const {
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  const auto config_copy = config;
  xSemaphoreGive(data_mutex);
  return config_copy;
}

void lorawan_handler::set_uplink_interval(const uint32_t interval) {
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  config.uplink_interval_min = interval;
  if (use_flash) {
    if (!save_config_to_flash())
      ESP_LOGE(tag.c_str(), "could not save to flash!");
  }
  xSemaphoreGive(data_mutex);
}

void lorawan_handler::set_join(const bool join) {
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  config.join = join;
  if (use_flash) {
    if (!save_config_to_flash())
      ESP_LOGE(tag.c_str(), "could not save to flash!");
  }
  xSemaphoreGive(data_mutex);
}

bool lorawan_handler::get_join() const {
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  const auto join_copy = config.join;
  xSemaphoreGive(data_mutex);
  return join_copy;
}

void lorawan_handler::handle_lorawan() {
  if (config.join) {
    if (!is_init) {
      init_lora_mac();
      is_init = true;
    }
    LmHandlerProcess();
    process_uplink();
  } else {
    if (is_init) {
      if (LoRaMacIsBusy()) {
        LmHandlerProcess();
        process_uplink();
      } else {
        if (deinit_lora_mac())
          is_init = false;
        else
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(5));
}

void lorawan_handler::set_values(const measurement_data &to_set) {
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  values = to_set;
  values_set = values.has_value();
  xSemaphoreGive(data_mutex);
}

std::array<uint8_t, 8> lorawan_handler::generate_eui() {
  std::array<uint8_t, 8> buffer;
  esp_fill_random(static_cast<void *>(&buffer), buffer.size());
  buffer[0] = (buffer[0] & ~1) | 2;
  return buffer;
}

std::array<uint8_t, 16> lorawan_handler::generate_key() {
  std::array<uint8_t, 16> buffer;
  esp_fill_random(static_cast<void *>(&buffer), buffer.size());
  return buffer;
}

void lorawan_handler::set_euis_and_key() {
  ESP_LOGI(tag.c_str(), "generate new EUIs and key");
  config.eui = generate_eui();
  config.join_eui = generate_eui();
  config.app_key = generate_key();
}

esp_err_t lorawan_handler::init_flash() {
  ESP_LOGI(tag.c_str(), "init flash memory");
  esp_err_t ret_value = nvs_flash_init();
  if (ret_value == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret_value == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGE(tag.c_str(), "error while init flash, retry after erase!");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret_value = nvs_flash_init();
  }
  return ret_value;
}

bool lorawan_handler::save_config_to_flash() {
  nvs_handle_t my_handle;

  esp_err_t err = nvs_open(STORAGE_NAME, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(tag.c_str(), "could not open flash storage, error: %s!",
             esp_err_to_name(err));
    return false;
  }

  err = nvs_set_blob(my_handle, LOCATION_NAME, static_cast<void *>(&config),
                     sizeof(lorawan_config));
  if (err == ESP_OK) {
    if (nvs_commit(my_handle) != ESP_OK)
      ESP_LOGE(tag.c_str(), "could not commit change!");
  } else
    ESP_LOGE(tag.c_str(), "could not set blob to flash storage, error: %s!",
             esp_err_to_name(err));

  nvs_close(my_handle);
  return !static_cast<bool>(err);
}

bool lorawan_handler::load_config_from_flash() {
  nvs_handle_t my_handle;
  esp_err_t err;

  err = nvs_open(STORAGE_NAME, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(tag.c_str(), "could not open flash storage, error: %s!",
             esp_err_to_name(err));
    return false;
  }

  lorawan_config to_load;
  size_t loaded_size = sizeof(to_load);
  err = nvs_get_blob(my_handle, LOCATION_NAME, static_cast<void *>(&to_load),
                     &loaded_size);

  if (err != ESP_OK)
    ESP_LOGE(tag.c_str(), "could not get blob from flash storage, error: %s!",
             esp_err_to_name(err));
  else if (loaded_size != sizeof(to_load))
    ESP_LOGE(tag.c_str(), "size of read data does not match size of config!");
  else
    config = to_load;

  nvs_close(my_handle);
  return !static_cast<bool>(err);
}

static void on_mac_mcps_request(LoRaMacStatus_t status, McpsReq_t *mcps_request,
                                TimerTime_t next_tx) {
  switch (mcps_request->Type) {
    case MCPS_CONFIRMED: {
      ESP_LOGI(tag_gl.c_str(), "Send confirm Packet, status: %s",
               MacStatusStrings[status]);
      break;
    }
    case MCPS_UNCONFIRMED: {
      ESP_LOGI(tag_gl.c_str(), "Send unconfirm Packet, status: %s",
               MacStatusStrings[status]);
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

static void on_mac_mlme_request(LoRaMacStatus_t status, MlmeReq_t *mlme_req,
                                TimerTime_t next_tx) {
  switch (mlme_req->Type) {
    case MLME_JOIN: {
      ESP_LOGI(tag_gl.c_str(), "Start join, status: %s",
               MacStatusStrings[status]);
      break;
    }
    case MLME_LINK_CHECK: {
      ESP_LOGI(tag_gl.c_str(), "Link check, status: %s",
               MacStatusStrings[status]);
      break;
    }
    case MLME_DEVICE_TIME: {
      ESP_LOGI(tag_gl.c_str(), "Device time, status: %s",
               MacStatusStrings[status]);
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

static void on_tx_timer_event(void *context) {
  if (values_set) is_tx_frame_pending = true;
  update_timer_periodicity();
}

static void on_join_request(LmHandlerJoinParams_t *params) {
  if (params->CommissioningParams->IsOtaaActivation == true) {
    if (params->Status == LORAMAC_HANDLER_SUCCESS)
      ESP_LOGI(tag_gl.c_str(), "Join Successful , DevAddr: %08lX",
               params->CommissioningParams->DevAddr);
    else
      ESP_LOGE(tag_gl.c_str(), "Join fail!");
  } else
    ESP_LOGI(tag_gl.c_str(), "Joined, DevAddr: %08lX",
             params->CommissioningParams->DevAddr);

  if (params->Status == LORAMAC_HANDLER_ERROR) {
    if (config.join) LmHandlerJoin();
  } else {
    if (LmHandlerRequestClass(CLASS_C) != LORAMAC_HANDLER_SUCCESS)
      ESP_LOGE(tag_gl.c_str(), "Request class fail!");

    on_tx_timer_event(nullptr);  // send data right now
  }
}

static std::string c_array_to_string(const uint8_t *array, const uint8_t size) {
  std::vector<uint8_t> data(array, array + size);
  std::stringstream string_result;
  string_result << std::hex << std::setfill('0');
  for (const auto &element : data) {
    string_result << std::setw(2) << static_cast<unsigned int>(element);
  }
  return string_result.str();
}

static void on_tx_data(LmHandlerTxParams_t *params) {
  MibRequestConfirm_t mibGet;

  std::string ack_text = "unconfirmed";

  if (params->AppData.BufferSize != 0 &&
      (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET)) {
    if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG) {
      if (params->AckReceived == 0) ESP_LOGW(tag_gl.c_str(), "NACK received!");
      ack_text = (params->AckReceived != 0) ? "confirm-ACK" : "confirm-NACK";
    }

    mibGet.Type = MIB_CHANNELS;
    LoRaMacMibGetRequestConfirm(&mibGet);
    ESP_LOGI(tag_gl.c_str(), "--> Uplink frame(%lu), FREQ:%lu, DR:%d, %s",
             params->UplinkCounter,
             mibGet.Param.ChannelList[params->Channel].Frequency,
             params->Datarate, ack_text.c_str());
    const auto data_string =
        c_array_to_string(params->AppData.Buffer, params->AppData.BufferSize);
    ESP_LOGI(tag_gl.c_str(), "Port:%d, data: %s", params->AppData.Port,
             data_string.c_str());
  }
}

static LmHandlerCallbacks_t lmhandler_callbacks = {
    .GetBatteryLevel = nullptr,
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
  tx_periodicity = config.uplink_interval_min * 60 * 1000 +
                   randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
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

void lorawan_handler::init_lora_mac() {
  ESP_LOGI(tag.c_str(), "Init LoRaMAC");
  if (LmHandlerInit(&lmhandler_callbacks, &lmhandler_params) !=
      LORAMAC_HANDLER_SUCCESS) {
    // Fatal error, endless loop.
    while (1) {
      ESP_LOGE(tag.c_str(), "LoRaMac wasn't properly initialized!");
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }

  MibRequestConfirm_t mib_req;

  // eui
  mib_req.Type = MIB_DEV_EUI;
  mib_req.Param.DevEui = config.eui.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  // join eui
  mib_req.Type = MIB_JOIN_EUI;
  mib_req.Param.JoinEui = config.join_eui.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  // app key
  mib_req.Type = MIB_APP_KEY;
  mib_req.Param.AppKey = config.app_key.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  mib_req.Type = MIB_NWK_KEY;
  mib_req.Param.NwkKey = config.app_key.data();
  if (LoRaMacMibSetRequestConfirm(&mib_req) != LORAMAC_STATUS_OK)
    ESP_LOGE(tag.c_str(), "Error while setting Dev EUI to LoRaMac!");

  LmHandlerSetSystemMaxRxError(50);
  LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &lmhpcomplience_params);

  LmHandlerJoin1(true);

  // TX timer
  calculate_default_periodicity();
  TimerInit(&tx_timer, on_tx_timer_event);
  TimerSetValue(&tx_timer, tx_periodicity);
}

bool lorawan_handler::deinit_lora_mac() {
  ESP_LOGI(tag.c_str(), "Deinit LoRaMAC");
  is_tx_frame_pending = false;
  TimerStop(&tx_timer);
  LoRaMacStop();
  LoRaMacStatus_t status = LoRaMacDeInitialization();
  if (status != LORAMAC_STATUS_OK) {
    ESP_LOGE(tag.c_str(), "LoRaMacDeInitialization error: %d", status);
    return false;
  }
  return true;
}

void lorawan_handler::process_uplink() {
  if (is_tx_frame_pending == true) prepare_tx_frame();
}

// send multiple packets
void lorawan_handler::prepare_tx_frame() {
  if (LmHandlerIsBusy() == true) return;

  if (packet_count >=
      static_cast<uint8_t>(value_types_lora::max_value_types_lora)) {
    packet_count = 0;
    is_tx_frame_pending = false;
    return;
  }

  u_int8_t index = 0;

  xSemaphoreTake(data_mutex, portMAX_DELAY);

  const auto values_to_send =
      get_values_for_type(static_cast<value_types_lora>(packet_count));

  if (values_to_send.empty()) {
    ESP_LOGE(tag.c_str(), "No values to send!");
    xSemaphoreGive(data_mutex);
    return;
  }

  app_data.Buffer[index] = packet_count;
  ++index;

  for (const auto &value_float : values_to_send) {
    uint32_t value_int = 0;
    static_assert(sizeof(float) == 4);
    memcpy(&value_int, &value_float, 4);
    app_data.Buffer[index++] = (value_int) & 0xff;
    app_data.Buffer[index++] = ((value_int) >> 8) & 0xff;
    app_data.Buffer[index++] = ((value_int) >> 16) & 0xff;
    app_data.Buffer[index++] = ((value_int) >> 24) & 0xff;
  }

  xSemaphoreGive(data_mutex);

  app_data.BufferSize = index;
  app_data.Port = LORAWAN_APP_PORT;

  if (LmHandlerSend(&app_data, lmhandler_params.IsTxConfirmed) ==
      LORAMAC_HANDLER_SUCCESS) {
    ESP_LOGI(tag.c_str(), "packet successfully sent!");
    ++packet_count;
  }
}

lorawan_handler::values_vector lorawan_handler::get_values_for_type(
    const value_types_lora &type) {
  const uint8_t type_as_int = static_cast<uint8_t>(type);
  ESP_LOGI(tag.c_str(), "get values for packet with type: %d", type_as_int);

  switch (type) {
    case value_types_lora::ir_camera_1_part_1:
    case value_types_lora::ir_camera_2_part_1:
    case value_types_lora::ir_camera_3_part_1:
    case value_types_lora::ir_camera_4_part_1:
    case value_types_lora::ir_camera_5_part_1:
    case value_types_lora::ir_camera_6_part_1:
      return get_pixels_for_camera(type_as_int / 2, false);

    case value_types_lora::ir_camera_1_part_2:
    case value_types_lora::ir_camera_2_part_2:
    case value_types_lora::ir_camera_3_part_2:
    case value_types_lora::ir_camera_4_part_2:
    case value_types_lora::ir_camera_5_part_2:
    case value_types_lora::ir_camera_6_part_2:
      return get_pixels_for_camera((type_as_int - 1) / 2, true);

    case value_types_lora::sensor_values:
      return get_values_combined();

    default:
      ESP_LOGE(tag.c_str(), "Undefined type!");
      return values_vector();
  }
}

lorawan_handler::values_vector lorawan_handler::get_pixels_for_camera(
    const uint8_t camera_index, const bool second_half) {
  if (!values.has_value()) {
    ESP_LOGE(tag.c_str(), "No values to handle!");
    return values_vector();
  }

  values_vector pixel_values;
  const uint8_t start = second_half ? 0 : PIXEL_COUNT / 2;
  const uint8_t end = second_half ? PIXEL_COUNT / 2 : PIXEL_COUNT;

  for (uint8_t pixel_index = start; pixel_index < end; ++pixel_index)
    pixel_values.push_back(
        values.value().ir_camera_data[camera_index][pixel_index]);

  return pixel_values;
}

lorawan_handler::values_vector lorawan_handler::get_values_combined() {
  if (!values.has_value()) {
    ESP_LOGE(tag.c_str(), "No values to handle!");
    return values_vector();
  }

  values_vector sensor_values;
  uint8_t index;

  for (index = 0;
       index <= static_cast<uint8_t>(combined_value_types::ir_camera_6_mean);
       ++index) {
    sensor_values.push_back(index);
    sensor_values.push_back(values.value().ir_camera_means[index]);
  }

  sensor_values.push_back(
      static_cast<float>(combined_value_types::air_temperature));
  sensor_values.push_back(values.value().air_temp);
  sensor_values.push_back(
      static_cast<float>(combined_value_types::air_humidity));
  sensor_values.push_back(values.value().air_rH);
  sensor_values.push_back(
      static_cast<float>(combined_value_types::surface_temperature));
  sensor_values.push_back(values.value().surface_temp);
  return sensor_values;
}