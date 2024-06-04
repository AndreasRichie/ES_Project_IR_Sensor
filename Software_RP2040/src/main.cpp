#include <Adafruit_AMG88xx.h>
#include <Adafruit_TMP117.h>
#include <Arduino.h>
#include <PacketSerial.h>
#include <SPI.h>
#include <SensirionErrors.h>
#include <SensirionI2cSht4x.h>
#include <TCA9548.h>
#include <Wire.h>

#include <array>
#include <string>

/******************************** Defines ********************************/

#define DEBUG 0

#define I2C_MUX_ADDRESS 0x70

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
#define PKT_TYPE_CMD_COLLECT_INTERVAL 0xA0
#define PKT_TYPE_CMD_BEEP_ON 0xA1
#define PKT_TYPE_CMD_SHUTDOWN 0xA3
#define PKT_TYPE_INVALID 0xFF

#define NUMBER_OF_CAMERAS 6

// size necessary for data of IR camera (64 pixels + type)
#define TRX_BUFFER_SIZE AMG88xx_PIXEL_ARRAY_SIZE * 4 + 1

/******************************* Variables *******************************/

Adafruit_AMG88xx amg8833;
TCA9548 i2_mux(I2C_MUX_ADDRESS);
SensirionI2cSht4x sht45;
Adafruit_TMP117 tmp117;

PacketSerial myPacketSerial;

bool ir_cameras_connected = false;
bool surface_temp_connected = false;
bool air_sensor_connected = false;

/*************************** Helper Functions ****************************/

// send sensor data to esp32
void sensor_data_send(uint8_t type, const std::vector<float>& data) {
  uint8_t data_buf[TRX_BUFFER_SIZE] = {0};
  int size = 0;

  data_buf[0] = type;
  size++;

  for (const auto& data_single : data) {
    memcpy(&data_buf[size], &data_single, sizeof(float));
    size += sizeof(float);
    if (size > TRX_BUFFER_SIZE) {
      Serial.println("Tried to send too many bytes!");
      return;
    }
  }

  myPacketSerial.send(data_buf, size);
#if DEBUG
  Serial.printf("---> send len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%x ", data_buf[i]);
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

uint8_t get_packet_type_from_index(const int index) {
  switch (index) {
    case 0:
      return PKT_TYPE_SENSOR_IR_CAMERA_1;
    case 1:
      return PKT_TYPE_SENSOR_IR_CAMERA_2;
    case 2:
      return PKT_TYPE_SENSOR_IR_CAMERA_3;
    case 3:
      return PKT_TYPE_SENSOR_IR_CAMERA_4;
    case 4:
      return PKT_TYPE_SENSOR_IR_CAMERA_5;
    case 5:
      return PKT_TYPE_SENSOR_IR_CAMERA_6;
    default:
      return PKT_TYPE_INVALID;
  }
}

/*************************** AMG8833 IR camera ***************************/
using pixel_values = std::array<float, AMG88xx_PIXEL_ARRAY_SIZE>;
std::array<pixel_values, 6> pixels_all;

void sensor_amg8833_init(void) {
#if DEBUG
  Serial.println(F("AMG88xx Thermal Camera!"));
#endif

  if (!amg8833.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    ir_cameras_connected = false;
    return;
  }

  ir_cameras_connected = true;
  Serial.println("-- Thermal Camera Connected --");
}

void sensor_amg8833_read(const int camera_index) {
  const uint8_t paket_type = get_packet_type_from_index(camera_index);
  if (paket_type == PKT_TYPE_INVALID) {
    Serial.println("Invalid Paket Type!");
    return;
  }

  amg8833.readPixels(pixels_all[camera_index].data());

#if DEBUG
  for (int pixel_index = 0; pixel_index < AMG88xx_PIXEL_ARRAY_SIZE;
       pixel_index++) {
    Serial.printf("AMG8833 %d Pixel %d: ", camera_index, pixel_index);
    Serial.println(pixels_all[camera_index][pixel_index]);
  }
#endif

  sensor_data_send(paket_type,
                   std::vector<float>(pixels_all[camera_index].begin(),
                                      pixels_all[camera_index].end()));
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

void sensor_tmp117_read(void) {
  // create an empty event to be filled
  sensors_event_t temp_event;
  // fill the empty event object with the current measurements
  tmp117.getEvent(&temp_event);

#if DEBUG
  Serial.print("TMP117 Temperature: ");
  Serial.println(temp_event.temperature);
#endif

  sensor_data_send(PKT_TYPE_SENSOR_SURF_TEMP, {temp_event.temperature});
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

void sensor_sht45_read(void) {
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

  sensor_data_send(PKT_TYPE_SENSOR_AIR_TEMP, {temperature});
  sensor_data_send(PKT_TYPE_SENSOR_AIR_HUM, {humidity});
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

void onPacketReceived(const uint8_t* buffer, size_t size) {
#if DEBUG
  Serial.printf("<--- recv len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%x ", buffer[i]);
  }
  Serial.println("");
#endif
  if (size < 1) {
    return;
  }
  switch (buffer[0]) {
    case PKT_TYPE_CMD_SHUTDOWN: {
      Serial.println("cmd shutdown");
      shutdown_flag = true;
      sensor_power_off();
      break;
    }
    default:
      break;
  }
}

/***************************** setup & loop ******************************/

int i = 0;

void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  sensor_power_on();

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();
  if (i2_mux.begin() == false) {
    Serial.println("Could not connect to I2C Multiplexer!");
  }

  // init all IR cameras
  for (unsigned int index = 0; index < NUMBER_OF_CAMERAS; ++index) {
    i2_mux.selectChannel(index);
    sensor_amg8833_init();
  }
  sensor_tmp117_init();
  sensor_sht45_init();

  beep_init();
  delay(500);
  beep_off();
}

void loop() {
  if (i > 500) {
    i = 0;
    // read from all IR cameras
    if (ir_cameras_connected)
      for (unsigned int index = 0; index < NUMBER_OF_CAMERAS; ++index) {
        i2_mux.selectChannel(index);
        sensor_amg8833_read(index);
        delay(50);
      }
    if (surface_temp_connected) sensor_tmp117_read();
    if (air_sensor_connected) sensor_sht45_read();
  }

  i++;

  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
    Serial.println("Buffer Overflow");
  }
  delay(10);
}