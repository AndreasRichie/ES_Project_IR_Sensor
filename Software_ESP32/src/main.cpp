#include <Arduino_GFX_Library.h>
#include <PCA95x5.h>
#include <PacketSerial.h>

PacketSerial_<COBS, 0, 512> myPacketSerial;

#define RXD2 20
#define TXD2 19

#define PKT_TYPE_SENSOR_IR_CAMERA_1 0XB0

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

  // myPacketSerial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  delay(5000);  // 5 seconds
}

void loop() {
  gfx->setCursor(random(gfx->width()), random(gfx->height()));
  gfx->setTextColor(random(0xffff), random(0xffff));
  gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */,
                   random(2) /* pixel_margin */);
  gfx->println("Sensecap Indicator");
  delay(1000);  // 1 second

  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
    Serial.println("Buffer Overflow");
  }
}

void onPacketReceived(const uint8_t *buffer, size_t size) {
  Serial.printf("<--- recv len:%d, data: ", size);

  if (size < 1) {
    return;
  }
  // byte serbytes[] = buffer[i];
  float dataval;
  uint8_t index;
  switch (buffer[0]) {
    case PKT_TYPE_SENSOR_IR_CAMERA_1: {
      index = buffer[1];
      memcpy(&dataval, &buffer[2], sizeof(float));
      Serial.printf("Camera 1 Result, Pixel %d: ", index);
      Serial.println(dataval);
      break;
    }
    default:
      break;
  }
}