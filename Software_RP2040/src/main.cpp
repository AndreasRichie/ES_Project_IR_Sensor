#include <Adafruit_AMG88xx.h>
#include <Adafruit_TMP117.h>
#include <Arduino.h>
#include <PacketSerial.h>
#include <SD.h>
#include <SPI.h>
#include <SensirionErrors.h>
#include <SensirionI2cSht4x.h>
#include <TCA9548.h>
#include <Wire.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>

/******************************** Defines ********************************/

#define DEBUG 0

#define DELAY_TIME 1
#define SEND_INTERVAL 2000
#define LOOP_COUNT ((SEND_INTERVAL - 500) / DELAY_TIME)

#define I2C_MUX_ADDRESS 0x70

// Type of transfer packet
#define PKT_TYPE_SENSOR_IR_CAMERA 0XB0
#define PKT_TYPE_SENSOR_SURF_TEMP 0XB1
#define PKT_TYPE_SENSOR_AIR_TEMP 0XB2
#define PKT_TYPE_SENSOR_AIR_HUM 0XB3
#define PKT_TYPE_MEAN_IR_CAMERA 0XB4
#define PKT_TYPE_MEAN_RADIANT_TEMP 0XB5
#define PKT_TYPE_CMD_SET_TIME 0xA0
#define PKT_TYPE_CMD_CALIBRATION 0XA1
#define PKT_TYPE_INVALID 0xFF

#define NUMBER_OF_CAMERAS 6

// IR data + index + calib type + calib ref + pkt type
#define CALIB_DATA_SIZE (AMG88xx_PIXEL_ARRAY_SIZE + 1) * sizeof(int16_t) + 3
// time_t + package type
#define TIMESTAMP_SIZE sizeof(time_t) + 1
// biggest size necessary for transmission is calibration data (64 pixels + calib type + calib ref + pkt type + index +
// overhead + delimiter) = 135 -> use next exp 2 value
#define RX_BUFFER_SIZE 256
#define TRX_BUFFER_SIZE RX_BUFFER_SIZE

// per line are saved date, time, camera_idx, calib_type, ref and the pixel values
#define CALIB_SAVE_COLS AMG88xx_PIXEL_ARRAY_SIZE + 5

#define KELVIN 273.15

enum calibration_type { manual, sensor };

using pixel_values = std::array<float, AMG88xx_PIXEL_ARRAY_SIZE>;
using camera_values = std::array<pixel_values, NUMBER_OF_CAMERAS>;

struct calibration_data {
  calibration_type type;
  float ref_value;
  pixel_values values = {0};
};

/******************************* Variables *******************************/

Adafruit_AMG88xx amg8833;
TCA9548 i2_mux(I2C_MUX_ADDRESS);
SensirionI2cSht4x sht45;
Adafruit_TMP117 tmp117;

PacketSerial_<COBS, 0, RX_BUFFER_SIZE> myPacketSerial;

std::array<bool, NUMBER_OF_CAMERAS> ir_cameras_connected = {false};
bool surface_temp_connected = false;
bool air_sensor_connected = false;

camera_values pixels_all_calc;
camera_values pixels_all_raw;
float tmp117_temp;
float sht45_temp;
float sht45_hum;

bool sd_init_flag = false;

// raw measurement data without calibration
static constexpr char log_file_name_raw[] = "measurement_data_raw.csv";
// measurement data including calibration and calculated values
static constexpr char log_file_name_calculated[] = "measurement_data_calculations.csv";
// calibration data
static constexpr char calibration_file_name[] = "calibration.csv";

bool time_set = false;

std::array<float, NUMBER_OF_CAMERAS> ir_mean_values = {0.f};
float mean_radiant_temp = 0.f;
std::array<calibration_data, NUMBER_OF_CAMERAS> calibration_all;

const size_t rows_and_cols = std::round(std::sqrt(AMG88xx_PIXEL_ARRAY_SIZE));

constexpr std::array<float, 64> solid_angles_pixels = {
    0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0162, 0.0162, 0.0162, 0.0162, 0.0162,
    0.0162, 0.0162, 0.0162, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0171, 0.0171,
    0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171, 0.0171,
    0.0171, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0168, 0.0162, 0.0162, 0.0162, 0.0162,
    0.0162, 0.0162, 0.0162, 0.0162, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154, 0.0154};

constexpr float view_factor = 1.f / 6.f;

const float sum_solid_angles = std::accumulate(solid_angles_pixels.cbegin(), solid_angles_pixels.cend(), 0.f);

/*************************** Helper Functions ****************************/

// send values to esp32
void send_value_data(const std::vector<uint8_t> type_info, const std::vector<float>& data) {
  // init buffer and size
  uint8_t data_buf[TRX_BUFFER_SIZE] = {0};
  int size = 0;

  // set packet type, index, calibration tye, ... whatever is provided
  for (const uint8_t& info_byte : type_info) {
    data_buf[size] = info_byte;
    ++size;
  }

  // iterate over all values
  for (const float& data_single : data) {
    // convert float to int16_t and copy bytes into buffer
    const int16_t data_int = data_single * 100.f;
    memcpy(&data_buf[size], &data_int, sizeof(int16_t));
    size += sizeof(int16_t);
    if (size > TRX_BUFFER_SIZE) {
      Serial.println("Tried to send too many bytes!");
      return;
    }
  }

  myPacketSerial.send(data_buf, size);
#if DEBUG
  Serial.printf("---> send len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%02x ", data_buf[i]);
  }
  Serial.println("");
#endif
}

void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

void sensor_power_off(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
}

std::string get_timestamp() {
  char timestamp_string[30];
  time_t timestamp = time(NULL);
  tm datetime = *localtime(&timestamp);

  strftime(timestamp_string, 30, "%d.%m.%Y,%T", &datetime);

  return std::string(timestamp_string);
}

/*************************** AMG8833 IR camera ***************************/

void sensor_amg8833_init(const int camera_index) {
#if DEBUG
  Serial.println(F("AMG88xx Thermal Camera!"));
#endif

  if (!amg8833.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    ir_cameras_connected[camera_index] = false;
    return;
  }

  ir_cameras_connected[camera_index] = true;
  Serial.println("-- Thermal Camera Connected --");
}

void correct_pixel_order(const int camera_index) {
  // the Adafruit library reads the pixels from left to right instead of the order that is specified in the AMG8833
  // datasheet, so swap the array
  for (size_t row = 0; row < rows_and_cols; ++row)
    for (size_t col = 0; col < rows_and_cols / 2; ++col)
      std::swap(pixels_all_raw[camera_index][8 * row + col], pixels_all_raw[camera_index][8 * (row + 1) - (col + 1)]);

  // reverse order of values, because pixels are read from lower right corner to upper left
  std::reverse(pixels_all_raw[camera_index].begin(), pixels_all_raw[camera_index].end());
}

void sensor_amg8833_read(const uint8_t camera_index) {
  amg8833.readPixels(pixels_all_raw[camera_index].data());
  correct_pixel_order(camera_index);

#if DEBUG
  for (int pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; pixel_index++) {
    Serial.printf("AMG8833 %d Pixel %d: ", camera_index, pixel_index);
    Serial.println(pixels_all_raw[camera_index][pixel_index]);
  }
#endif

  for (size_t pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
    pixels_all_calc[camera_index][pixel_index] =
        pixels_all_raw[camera_index][pixel_index] + calibration_all[camera_index].values[pixel_index];

  send_value_data({PKT_TYPE_SENSOR_IR_CAMERA, camera_index},
                  std::vector<float>(pixels_all_calc[camera_index].begin(), pixels_all_calc[camera_index].end()));
}

void calculate_ir_camera_mean(const uint8_t camera_index) {
  float sum = 0.f;

  for (size_t pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
    sum += std::pow((pixels_all_calc[camera_index][pixel_index] + KELVIN), 4) * solid_angles_pixels[pixel_index];

  ir_mean_values[camera_index] = std::pow(sum, 0.25f) - KELVIN;

  send_value_data({PKT_TYPE_MEAN_IR_CAMERA, camera_index}, {ir_mean_values[camera_index]});
}

void calculate_mean_radiant_temperature() {
  float sum = 0.f;

  for (size_t camera_index = 0; camera_index < NUMBER_OF_CAMERAS; ++camera_index)
    sum += std::pow((ir_mean_values[camera_index] + KELVIN), 4) * view_factor;

  mean_radiant_temp = std::pow(sum, 0.25f) - KELVIN;

  send_value_data({PKT_TYPE_MEAN_RADIANT_TEMP}, {mean_radiant_temp});
}

/******************* TMP117 Surface Temperature Sensor *******************/
void sensor_tmp117_init(void) {
#if DEBUG
  Serial.println(F("TMP117 Temperature Sensor!"));
#endif

  if (!tmp117.begin()) {
    Serial.println("Could not find a valid TMP117 sensor, check wiring!");
    surface_temp_connected = false;
    return;
  }

  surface_temp_connected = true;
  Serial.println("-- Surface Temperature Sensor Connected --");
}

void sensor_tmp117_read() {
  // create an empty event to be filled
  sensors_event_t temp_event;
  // fill the empty event object with the current measurements
  tmp117.getEvent(&temp_event);

#if DEBUG
  Serial.print("TMP117 Temperature: ");
  Serial.println(temp_event.temperature);
#endif

  tmp117_temp = temp_event.temperature;

  send_value_data({PKT_TYPE_SENSOR_SURF_TEMP}, {temp_event.temperature});
}

/*********************** SHT45 Air T and rH Sensor ***********************/
char error_message[64];
int16_t error;

void sensor_sht45_init(void) {
#if DEBUG
  Serial.println(F("SHT45 Temperature and Humidity Sensor!"));
#endif

  sht45.begin(Wire, SHT40_I2C_ADDR_44);

  error = sht45.softReset();

  if (error != NoError) {
    Serial.print("Error trying to do soft reset: ");
    errorToString(error, error_message, sizeof error_message);
    Serial.println(error_message);
    air_sensor_connected = false;
    return;
  }
  delay(10);

  uint32_t serialNumber = 0;
  error = sht45.serialNumber(serialNumber);
  if (error != NoError) {
    Serial.print("Error trying to read serial number: ");
    errorToString(error, error_message, sizeof(error_message));
    Serial.println(error_message);
    air_sensor_connected = false;
    return;
  }

#if DEBUG
  Serial.print("serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();
#endif

  air_sensor_connected = true;
  Serial.println("-- Air Temperature and Humidity Sensor Connected --");
}

void sensor_sht45_read() {
  float temperature = 0.0;
  float humidity = 0.0;
  delay(20);
  error = sht45.measureHighPrecision(temperature, humidity);
  if (error != NoError) {
    Serial.print("Error trying to read sensor data: ");
    errorToString(error, error_message, sizeof(error_message));
    Serial.println(error_message);
    return;
  }

#if DEBUG
  Serial.print("SHT45 Temperature: ");
  Serial.print(temperature);
  Serial.print(", Humidity: ");
  Serial.println(humidity);
#endif

  sht45_temp = temperature;
  sht45_hum = humidity;

  send_value_data({PKT_TYPE_SENSOR_AIR_TEMP}, {temperature});
  send_value_data({PKT_TYPE_SENSOR_AIR_HUM}, {humidity});
}

/********************************* beep **********************************/

#define Buzzer 19  // Buzzer GPIO

void beep_init(void) { pinMode(Buzzer, OUTPUT); }
void beep_off(void) { digitalWrite(19, LOW); }
void beep_on(void) {
  analogWrite(Buzzer, 127);
  delay(50);
  analogWrite(Buzzer, 0);
}

/************************** recv cmd from esp32 **************************/

static bool shutdown_flag = false;

bool check_decoded_data_size(const size_t size, const size_t expected_size, const std::string& value_type) {
  if (size != expected_size) {
    Serial.printf(
        "Did not receive the correct ammount of bytes for %s! Expected: %d, "
        "received: %d\n",
        value_type.c_str(), expected_size, size);
    return false;
  }
  return true;
}

void handle_timestamp_received(const uint8_t* buffer, const size_t size) {
  Serial.print("Set current system time to ");

  if (!check_decoded_data_size(size, TIMESTAMP_SIZE, "timestamp")) return;
  timeval to_set;
  memcpy(&to_set.tv_sec, &buffer[1], sizeof(time_t));
  to_set.tv_usec = 0;

  char time_string[30];
  tm datetime = *localtime(&to_set.tv_sec);
  strftime(time_string, 30, "%d.%m.%Y,%T", &datetime);
  Serial.printf("%s\n", time_string);

  settimeofday(&to_set, nullptr);
  time_set = true;
}

// forward declare
void save_calibration_values_to_file(const uint8_t index);

float convert_bytes_to_float(const uint8_t* bytes) {
  int16_t data_value;
  memcpy(&data_value, bytes, sizeof(int16_t));
  return static_cast<float>(data_value) / 100.f;
}

void handle_calibration_received(const uint8_t* buffer, const size_t size) {
  size_t size_converted = 1;
  if (!check_decoded_data_size(size, CALIB_DATA_SIZE, "calibration")) {
    // problem highly possible due to unfinished transition before due to small hardware RX buffer of RP2040 --> skip
    // the incomplete paket at the beginning
    while (buffer[size_converted] != PKT_TYPE_CMD_CALIBRATION) ++size_converted;
    ++size_converted;
  }
  uint8_t camera_index = buffer[size_converted];
  if (camera_index > 5) {
    Serial.println("An invalid camera index was encountered!");
    return;
  }
  ++size_converted;
  uint8_t type = buffer[size_converted];
  if (type > 1) {
    Serial.println("An invalid calibration type was encountered!");
    return;
  }
  ++size_converted;
  calibration_all[camera_index].type = static_cast<calibration_type>(type);
  calibration_all[camera_index].ref_value = convert_bytes_to_float(&buffer[size_converted]);
  size_converted += sizeof(int16_t);
  size_t pixel_index = 0;
  while (size_converted < size) {
    calibration_all[camera_index].values[pixel_index] = convert_bytes_to_float(&buffer[size_converted]);
    size_converted += sizeof(int16_t);
    ++pixel_index;
  }
  if (pixel_index != AMG88xx_PIXEL_ARRAY_SIZE) Serial.println("Not all pixels were sent!");
  save_calibration_values_to_file(camera_index);
}

void onPacketReceived(const uint8_t* buffer, size_t size) {
#if DEBUG
  if (size > 0) {
    Serial.printf("<--- recv len:%d, data: ", size);
    for (int i = 0; i < size; i++) {
      Serial.printf("0x%02x ", buffer[i]);
    }
    Serial.println("");
  }
#endif

  if (size < 1) {
    return;
  }

  Serial.println("received packet");

  switch (buffer[0]) {
    // case PKT_TYPE_CMD_SHUTDOWN:
    //   Serial.println("cmd shutdown");
    //   shutdown_flag = true;
    //   sensor_power_off();
    //   break;
    case PKT_TYPE_CMD_SET_TIME:
      handle_timestamp_received(buffer, size);
      break;
    case PKT_TYPE_CMD_CALIBRATION:
      handle_calibration_received(buffer, size);
      break;
    default:
      Serial.println("Received invalid data!");
  }
}

/***************************** file saving *******************************/

bool init_spi_sd_card() {
  pinMode(7, INPUT);
  PinStatus card_detect = digitalRead(7);
  if (card_detect == HIGH) {
    Serial.println("No Card present!");
    return false;
  }
  Serial.println("-- Card detected! --");

  const int chipSelect = 13;
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  if (!SD.begin(chipSelect, 5000000, SPI1)) {
    Serial.println("Card begin failed!");
    return false;
  }
  Serial.println("-- Card initialized. --");
  return true;
}

bool is_free_space() {
  FSInfo fs_info;
  SDFS.info(fs_info);
  if (fs_info.totalBytes - fs_info.usedBytes < 10000) return false;
  return true;
}

std::optional<File> open_and_check_file(const std::string& file_name, const bool write) {
  File file = File();
  if (write)
    file = SD.open(file_name.c_str(), FILE_WRITE);
  else
    file = SD.open(file_name.c_str(), FILE_READ);

  // check if file is successfully opened/created
  if (!file) {
    Serial.printf("Could not open file %s!\n", file_name.c_str());
    return std::optional<File>();
  }
  return file;
}

std::optional<File> create_new_file(const std::string& file_name) {
  // if it already exists do nothing
  if (SD.exists(file_name.c_str())) return std::optional<File>();

  // if not, create (open) it
  return open_and_check_file(file_name, true);
}

void create_file_with_header_if_not_exists(const std::string& file_name, const std::string& header) {
  // check if file already exists and if it can be opened
  std::optional<File> file = create_new_file(file_name);

  // file already exists or can not be accessed
  if (!file.has_value()) return;

  file->printf("%s\n", header.c_str());
  file->close();
}

std::stringstream get_header_base() {
  std::stringstream header("");
  header.fill('0');
  header.width(2);
  header << "date,time,";
  return header;
}

std::string get_raw_logging_header() {
  std::stringstream log_raw_header = get_header_base();

  for (unsigned int camera_index = 0; camera_index < NUMBER_OF_CAMERAS; ++camera_index)
    for (unsigned int pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
      log_raw_header << "camera" << camera_index << "_pixel" << pixel_index << ",";

  log_raw_header << "surface_temp,air_temp,air_hum";

  return log_raw_header.str();
}

std::string get_calculated_logging_header() {
  std::stringstream log_calc_header = get_header_base();

  for (unsigned int camera_index = 0; camera_index < NUMBER_OF_CAMERAS; ++camera_index)
    for (unsigned int pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
      log_calc_header << "calib_camera" << camera_index << "_pixel" << pixel_index << ",";

  log_calc_header << "surface_temp,air_temp,air_hum,";
  for (unsigned int camera_index = 0; camera_index < NUMBER_OF_CAMERAS; ++camera_index)
    log_calc_header << "camera" << camera_index << "_mean,";

  log_calc_header << "mean_rad_temp";

  return log_calc_header.str();
}

std::string get_calibration_header() {
  std::stringstream calib_header = get_header_base();

  calib_header << "cam_idx,calib_type,calib_ref";

  for (unsigned int pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
    calib_header << ",pixel" << pixel_index;

  return calib_header.str();
}

void log_zeros(std::stringstream& log_string, const int number_of_zeros) {
  for (unsigned int count = 0; count < number_of_zeros; ++count) log_string << ",0";
}

void create_log_string_base(std::stringstream& string, const camera_values& cameras) {
  string.setf(std::ios::fixed);
  string.precision(2);
  // set timestamp to string
  string << get_timestamp();

  for (uint8_t index = 0; index < NUMBER_OF_CAMERAS; ++index) {
    if (ir_cameras_connected[index])
      for (const float& pixel : cameras[index]) string << "," << pixel;
    else
      log_zeros(string, AMG88xx_PIXEL_ARRAY_SIZE);
  }
  if (surface_temp_connected)
    string << "," << tmp117_temp;
  else
    log_zeros(string, 1);
  if (air_sensor_connected)
    string << "," << sht45_temp << "," << sht45_hum;
  else
    log_zeros(string, 2);
}

std::string create_raw_log_string() {
  std::stringstream log_raw_string("");
  create_log_string_base(log_raw_string, pixels_all_raw);
  return log_raw_string.str();
}

std::string create_calc_log_string() {
  std::stringstream log_calc_string("");
  create_log_string_base(log_calc_string, pixels_all_calc);

  for (uint8_t index = 0; index < NUMBER_OF_CAMERAS; ++index) log_calc_string << "," << ir_mean_values[index];

  log_calc_string << "," << mean_radiant_temp;
  return log_calc_string.str();
}

bool check_file_save_prerequisites() {
  // only try to write to SD card if it is initialized
  if (!sd_init_flag) {
    Serial.println("Tried to write to file, but SD not initilized!");
    return false;
  }
  // TODO: maybe change to still saving with no time set but marking it
  // only try to write to SD card if current time is set
  if (!time_set) {
    Serial.println("Tried to write to file, but no time set!");
    return false;
  }
  return true;
}

void write_to_file(const std::string& file_name, const std::string& data_string) {
  if (!check_file_save_prerequisites()) return;

  // open file
  std::optional<File> file = open_and_check_file(file_name, true);
  // check if file is available
  if (!file.has_value()) return;

  file->printf("%s\n", data_string.c_str());
  file->close();
}

void save_calibration_values_to_file(const uint8_t index) {
  std::stringstream calib_string("");
  calib_string.setf(std::ios::fixed);
  calib_string.precision(2);
  // set timestamp to string
  calib_string << get_timestamp();

  calib_string << "," << std::to_string(index) << "," << static_cast<unsigned int>(calibration_all[index].type) << ","
               << calibration_all[index].ref_value;

  for (unsigned int pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
    calib_string << "," << calibration_all[index].values[pixel_index];

  write_to_file(calibration_file_name, calib_string.str());
}

std::vector<std::string> split_string(const std::string& to_split) {
  std::vector<std::string> tokens;
  std::string to_split_copy = to_split;
  size_t pos = 0;
  std::string token;
  while ((pos = to_split_copy.find(',')) != std::string::npos) {
    token = to_split_copy.substr(0, pos);
    tokens.push_back(token);
    to_split_copy.erase(0, pos + 1);
  }
  tokens.push_back(to_split_copy);

  return tokens;
}

void read_calib_from_file() {
  Serial.println("Read data from calib file...");
  // open calib file
  std::optional<File> file = open_and_check_file(calibration_file_name, false);
  // check if file is available
  if (!file.has_value()) return;

  std::vector<std::string> read_lines;

  // read all lines to buffer
  while (file->available()) {
    read_lines.push_back(file->readStringUntil('\n').c_str());
  }

  if (read_lines.size() < 2) {
    Serial.println("No data to read in calibration file!");
    return;
  }

  // reverse order to read from back to front
  std::reverse(read_lines.begin(), read_lines.end());
  // remove header
  read_lines.erase(read_lines.end());

  // keep check of which cameras are already calibrated with read values
  std::array<bool, NUMBER_OF_CAMERAS> is_calibrated = {false};

  for (const std::string& line : read_lines) {
    std::vector<std::string> tokens = split_string(line);

    if (tokens.size() < CALIB_SAVE_COLS) {
      Serial.println("Read line not parsed correctly!");
      continue;
    }
    const int camera_index = std::stoi(tokens[2]);
    if (is_calibrated[camera_index]) continue;

    calibration_all[camera_index].type = static_cast<calibration_type>(std::stoi(tokens[3]));
    calibration_all[camera_index].ref_value = std::stof(tokens[4]);

    for (size_t pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE; ++pixel_index)
      calibration_all[camera_index].values[pixel_index] = std::stof(tokens[pixel_index + 5]);

    is_calibrated[camera_index] = true;

    if (std::all_of(is_calibrated.cbegin(), is_calibrated.cend(), [](const bool value) { return value == true; }))
      break;
  }

  for (size_t camera_index = 0; camera_index < NUMBER_OF_CAMERAS; ++camera_index) {
    std::vector<float> calibration_values(calibration_all[camera_index].values.begin(),
                                          calibration_all[camera_index].values.end());
    calibration_values.insert(calibration_values.begin(), calibration_all[camera_index].ref_value);

    send_value_data({PKT_TYPE_CMD_CALIBRATION, static_cast<uint8_t>(camera_index),
                     static_cast<uint8_t>(calibration_all[camera_index].type)},
                    calibration_values);
  }
  Serial.println("Successfully read data from calib file!");
}

/***************************** setup & loop ******************************/

int i = 0;

void setup() {
  Serial.begin(115200);

  // Wait for serial port to connect or 5 seconds if no debug serial is
  // used.
  int wait_count = 0;
  while (!Serial && wait_count < 5000) {
    ++wait_count;
    delay(1);
  }

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  sensor_power_on();

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  sd_init_flag = init_spi_sd_card();
  if (!sd_init_flag)
    Serial.println("SD Card not initialized and will not be used!");
  else {
    create_file_with_header_if_not_exists(log_file_name_raw, get_raw_logging_header());
    create_file_with_header_if_not_exists(log_file_name_calculated, get_calculated_logging_header());
    create_file_with_header_if_not_exists(calibration_file_name, get_calibration_header());
  }

  if (i2_mux.begin() == false) {
    Serial.println("Could not connect to I2C Multiplexer!");
  }

  // init all IR cameras
  for (unsigned int index = 0; index < NUMBER_OF_CAMERAS; ++index) {
    i2_mux.selectChannel(index);
    sensor_amg8833_init(index);
  }
  sensor_tmp117_init();
  sensor_sht45_init();

  beep_init();
  beep_off();

  read_calib_from_file();
}

void loop() {
  if (i > LOOP_COUNT) {
    i = 0;

    // read from all IR cameras
    for (uint8_t index = 0; index < NUMBER_OF_CAMERAS; ++index) {
      if (ir_cameras_connected[index]) {
        i2_mux.selectChannel(index);
        sensor_amg8833_read(index);
        delay(50);
        calculate_ir_camera_mean(index);
      }
    }
    calculate_mean_radiant_temperature();
    if (surface_temp_connected) sensor_tmp117_read();
    if (air_sensor_connected) sensor_sht45_read();

    write_to_file(log_file_name_raw, create_raw_log_string());
    write_to_file(log_file_name_calculated, create_calc_log_string());
  }

  i++;

  // TODO either let this run on a different core or implement a software RX buffer that is filled on interrupt -->
  // because of the small hardware RX buffer larger packets might not be received completely due to measurement duration
  // or use update in interrupt handler (it already implements a software buffer actually)
  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
    Serial.println("Buffer Overflow");
  }
  delay(DELAY_TIME);
}