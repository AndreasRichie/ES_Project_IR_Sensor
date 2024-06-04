#include <Arduino_GFX_Library.h>
#include <PCA95x5.h>
#include <PacketSerial.h>

#include <numeric>

#include "measurement_data.h"

#define DEBUG 0

#define RXD2 20
#define TXD2 19

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

#define GFX_BL \
  DF_GFX_BL  // default backlight pin, you may replace DF_GFX_BL to actual
             // backlight pin

/* More dev device declaration:
 * https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration
 */
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */

#define GFX_DEV_DEVICE ESP32_S3_RGB
#define GFX_BL 45
Arduino_DataBus *bus =
    new Arduino_SWSPI(GFX_NOT_DEFINED /* DC */, PCA95x5::Port::P04 /* CS */,
                      41 /* SCK */, 48 /* MOSI */, GFX_NOT_DEFINED /* MISO
                      */);

// option 1:
// Uncomment for 4" rect display
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    18 /* DE */, 17 /* VSYNC */, 16 /* HSYNC */, 21 /* PCLK */, 4 /* R0 */,
    3 /* R1 */, 2 /* R2 */, 1 /* R3 */, 0 /* R4 */, 10 /* G0 */, 9 /* G1 */,
    8 /* G2 */, 7 /* G3 */, 6 /* G4 */, 5 /* G5 */, 15 /* B0 */, 14 /* B1 */,
    13 /* B2 */, 12 /* B3 */, 11 /* B4 */, 1 /* hsync_polarity */,
    10 /* hsync_front_porch */, 8 /* hsync_pulse_width */,
    50 /* hsync_back_porch */, 1 /* vsync_polarity */,
    10 /* vsync_front_porch */, 8 /* vsync_pulse_width */,
    20 /* vsync_back_porch */);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 2 /* rotation */,
    true /* auto_flush */, bus, GFX_NOT_DEFINED /* RST */,
    st7701_type1_init_operations, sizeof(st7701_type1_init_operations));

#endif /* !defined(DISPLAY_DEV_KIT) */
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

PacketSerial_<COBS, 0, 512> myPacketSerial;
measurement_data data;

void onPacketReceived(const uint8_t *buffer, size_t size);

void setup(void) {
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX Hello World example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  gfx->setCursor(10, 10);
  gfx->setTextColor(RED);
  gfx->println("Sensecap Indicator");

  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  delay(5000);  // 5 seconds
}

int count = 0;

void loop() {
  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
    Serial.println("Buffer Overflow");
  }

  if (count >= 100) {
    count = 0;

#if DEBUG
    data.print_to_serial(false);
#endif

    gfx->setCursor(random(gfx->width()), random(gfx->height()));
    gfx->setTextColor(random(0xffff), random(0xffff));
    gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */,
                     random(2) /* pixel_margin */);
    gfx->println("Sensecap Indicator");
  }

  ++count;
  delay(10);  // 10 milliseconds
}

void handle_camera_received(const uint8_t *buffer, const int camera_index,
                            const size_t &size) {
  size_t size_converted = 1;
  unsigned int pixel_index = 0;
  while (size_converted < size) {
    float dataval;
    memcpy(&dataval, &buffer[size_converted], sizeof(float));
    data.ir_camera_data[camera_index][pixel_index] = dataval;
    size_converted += sizeof(float);
    ++pixel_index;
  }
  if (pixel_index != PIXEL_COUNT) {
    Serial.println("Not all pixels were sent!");
    return;
  }
  data.ir_camera_means[camera_index] =
      std::accumulate(data.ir_camera_data[camera_index].cbegin(),
                      data.ir_camera_data[camera_index].cend(), 0.0) /
      PIXEL_COUNT;
}

void onPacketReceived(const uint8_t *buffer, size_t size) {
  if (size < 1) {
    return;
  }
  switch (buffer[0]) {
    case PKT_TYPE_SENSOR_IR_CAMERA_1: {
      handle_camera_received(buffer, 0, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_2: {
      handle_camera_received(buffer, 1, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_3: {
      handle_camera_received(buffer, 2, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_4: {
      handle_camera_received(buffer, 3, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_5: {
      handle_camera_received(buffer, 4, size);
      break;
    }
    case PKT_TYPE_SENSOR_IR_CAMERA_6: {
      handle_camera_received(buffer, 5, size);
      break;
    }
    case PKT_TYPE_SENSOR_SURF_TEMP: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &buffer[1], sizeof(float));
      data.surface_temp = dataval;
      break;
    }
    case PKT_TYPE_SENSOR_AIR_TEMP: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &buffer[1], sizeof(float));
      data.air_temp = dataval;
      break;
    }
    case PKT_TYPE_SENSOR_AIR_HUM: {
      float dataval;
      // index only necessary for IR cameras, skipped
      memcpy(&dataval, &buffer[1], sizeof(float));
      data.air_rH = dataval;
      break;
    }
    default:
      Serial.println("Invalid Paket received!");
      break;
  }
}