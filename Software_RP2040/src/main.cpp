#include <Adafruit_AMG88xx.h>
#include <Arduino.h>
#include <PacketSerial.h>
#include <SPI.h>
#include <TCA9548.h>
#include <Wire.h>

#define DEBUG 0
#define I2C_MUX_ADDRESS 0x70

Adafruit_AMG88xx amg;
TCA9548 i2_mux(I2C_MUX_ADDRESS);

PacketSerial_<COBS, 0, 512> myPacketSerial;

// Type of transfer packet

#define PKT_TYPE_SENSOR_IR_CAMERA_1 0XB0
#define PKT_TYPE_CMD_COLLECT_INTERVAL 0xA0
#define PKT_TYPE_CMD_BEEP_ON 0xA1
#define PKT_TYPE_CMD_SHUTDOWN 0xA3

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

// sensor data send to  esp32
void sensor_data_send(uint8_t type, uint8_t index, float data) {
  uint8_t data_buf[32] = {0};
  int size = 0;

  data_buf[0] = type;
  size++;

  data_buf[1] = index;
  size++;

  memcpy(&data_buf[2], &data, sizeof(float));
  size += sizeof(float);

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

/************************ amg8833 IR camera  ****************************/

void sensor_amg8833_init(void) {
  Serial.println(F("AMG88xx thermal camera!"));

  bool status;

  status = amg.begin();
  if (!status) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Thermal Camera Test --");
}

void sensor_amg8833_get(void) {
  amg.readPixels(pixels);

  for (int index = 0; index < AMG88xx_PIXEL_ARRAY_SIZE; index++) {
    Serial.printf("AMG8833 Pixel %d: ", index);
    Serial.println(pixels[index]);

    sensor_data_send(PKT_TYPE_SENSOR_IR_CAMERA_1, static_cast<uint8_t>(index),
                     static_cast<float>(pixels[index]));
  }
}

/************************ beep ****************************/

#define Buzzer 19  // Buzzer GPIO

void beep_init(void) { pinMode(Buzzer, OUTPUT); }
void beep_off(void) { digitalWrite(19, LOW); }
void beep_on(void) {
  analogWrite(Buzzer, 127);
  delay(50);
  analogWrite(Buzzer, 0);
}

/************************ recv cmd from esp32  ****************************/

static bool shutdown_flag = false;

void onPacketReceived(const uint8_t *buffer, size_t size) {
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

/************************ setuo & loop ****************************/

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

  i2_mux.selectChannel(2);
  sensor_amg8833_init();

  beep_init();
  delay(500);
  beep_off();
}

void loop() {
  if (i > 500) {
    i = 0;

    Serial.print("\r\n\r\n--------- start measure -------\r\n");

    i2_mux.selectChannel(2);
    sensor_amg8833_get();
  }

  i++;

  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
    Serial.println("Buffer Overflow");
  }
  delay(10);
}