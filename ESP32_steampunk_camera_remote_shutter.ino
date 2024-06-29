/*
  Smartphone camera remote shutter trigger button

  The only thing it does is described in the header. Although there are a view tweakable parameters in settings.h.
  Tested with Open Camera on Android 8.0.
  It possible works with Iphone either.

  Ressources:
  Bluetooth HID Profile, Page 20: https://www.silabs.com/documents/login/application-notes/AN993.pdf
  USB HID Usage Tables, Page 53: https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf
  Bluetooth.org Specifications https://www.bluetooth.org/en-us/specification/adopted-specifications

  Hardware:
  ESP32 mini, or any other ESP32.
  Battery compartment 4.5V (3xAAA).
  Tactile switch button.
  Pull up resistor 10k to 50k will work.
  Double sided adhesive tape.

  Author: Michael Ruck, 9. Juny 2019
  License: GNU Lesser General Public License version 3 (https://opensource.org/licenses/LGPL-3.0)

*/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "./settings.h"


//// NEOPIXELS
#include <Adafruit_NeoPixel.h>
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(__NEOPIXEL_COUNT, __NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
enum LED_MODES { OFF, WAITING, SENDING, ANTICIPATING };
LED_MODES ledMode = WAITING;
bool ledRestart = false; // allows for quick mode change, ends loops

// on-board LED to indicate bluetooth activity
#define LED 2

BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;

bool connected = false;
bool btnFlag = false;

class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      connected = true;
      BLE2902* desc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc->setNotifications(true);
      Serial.printf("Connected to %s\n", desc->getUUID().toString());
    }

    void onDisconnect(BLEServer* pServer) {
      connected = false;
      BLE2902* desc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc->setNotifications(false);
      Serial.printf("Disconnected from %s\n", desc->getUUID().toString());
    }
};

/*
   This callback is connect with output report. In keyboard output report report special keys changes, like CAPSLOCK, NUMLOCK
   We can add digital pins with LED to show status
   bit 0 - NUM LOCK
   bit 1 - CAPS LOCK
   bit 2 - SCROLL LOCK
*/
class MyOutputCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* me) {
      uint8_t* value = (uint8_t*)(me->getValue().c_str());
      ESP_LOGI(LOG_TAG, "special keys: %d", *value);
    }
};

void taskServer(void*) {


  BLEDevice::init(__BT_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());

  hid = new BLEHIDDevice(pServer);
  input = hid->inputReport(1); // <-- input REPORTID from report map
  output = hid->outputReport(1); // <-- output REPORTID from report map

  output->setCallbacks(new MyOutputCallbacks());

  String name = __MANUFACTURER;
  hid->manufacturer()->setValue(name);

  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x02);

  BLESecurity *pSecurity = new BLESecurity();
  //  pSecurity->setKeySize();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  const uint8_t report[] = {
    USAGE_PAGE(1),      0x01,       // Generic Desktop Ctrls
    USAGE(1),           0x06,       // Keyboard
    COLLECTION(1),      0x01,       // Application
    REPORT_ID(1),       0x01,        //   Report ID (1)
    USAGE_PAGE(1),      0x07,       //   Kbrd/Keypad
    USAGE_MINIMUM(1),   0xE0,
    USAGE_MAXIMUM(1),   0xE7,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1),     0x01,       //   1 byte (Modifier)
    REPORT_COUNT(1),    0x08,
    HIDINPUT(1),           0x02,       //   Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position
    REPORT_COUNT(1),    0x01,       //   1 byte (Reserved)
    REPORT_SIZE(1),     0x08,
    HIDINPUT(1),           0x01,       //   Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position
    REPORT_COUNT(1),    0x06,       //   6 bytes (Keys)
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,       //   101 keys
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    HIDINPUT(1),           0x00,       //   Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position
    REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x08,       //   LEDs
    USAGE_MINIMUM(1),   0x01,       //   Num Lock
    USAGE_MAXIMUM(1),   0x05,       //   Kana
    HIDOUTPUT(1),          0x02,       //   Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile
    REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
    REPORT_SIZE(1),     0x03,
    HIDOUTPUT(1),          0x01,       //   Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile
    END_COLLECTION(0)
  };

  hid->reportMap((uint8_t*)report, sizeof(report));
  hid->startServices();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_KEYBOARD);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->start();
  hid->setBatteryLevel(7);

  ESP_LOGD(LOG_TAG, "Advertising started!");
  delay(portMAX_DELAY);

};


void neopixelServer(void*) {
 for(;;)
    {

        switch(ledMode) {
        case WAITING:
            rainbow(40);
            break;
        case SENDING:
            rainbow(10);
            break;
        case ANTICIPATING:
            theaterChaseRainbow(50);
            break;
        }
    }
}


// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait)
{
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for (long firstPixelHue = 0; !ledRestart && firstPixelHue < 5 * 65536; firstPixelHue += 256)
  {
    for (int i = 0; !ledRestart && i < strip.numPixels(); i++)
    { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}


// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait)
{
  int firstPixelHue = 0; // First pixel starts at red (hue 0)
  for (int a = 0; !ledRestart && a < 30; a++)
  { // Repeat 30 times...
    for (int b = 0; !ledRestart && b < 3; b++)
    { //  'b' counts from 0 to 2...
      strip.clear(); //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for (int c = b; !ledRestart && c < strip.numPixels(); c += 3)
      {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int hue = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color);                       // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
void colorWipe(uint32_t color, int wait)
{
  for (int i = 0; !ledRestart && i < strip.numPixels(); i++)
  { // For each pixel in strip...
    strip.setPixelColor(i, color); //  Set pixel's color (in RAM)
    strip.show();                  //  Update strip to match
    delay(wait);                   //  Pause for a moment
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE works!");

  // The Button
  pinMode(__BUTTONPIN_UP, INPUT_PULLUP);
  pinMode(__BUTTONPIN_DOWN, INPUT_PULLUP);
  setupInterrupts();

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // NeoPixel
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();  // Turn OFF all pixels ASAP
  pinMode(__NEOPIXEL_PIN, OUTPUT);
  digitalWrite(__NEOPIXEL_PIN, LOW);
  changeLedMode(WAITING);

  xTaskCreate(taskServer, "bluetoothServer", 20000, NULL, 5, NULL);
  xTaskCreate(neopixelServer, "neopixelServer", 20000, NULL, 10, NULL);
}

void loop() {

  Serial.printf("Loop is still alive, flags: %d, %d, %d ", connected, btnFlag, ledRestart);
  if (connected & btnFlag) {
    btnFlag = false;
    changeLedMode(SENDING);
    digitalWrite(LED, HIGH);
    Serial.println("Cheese...");

    //Key press
    uint8_t msg[] = {0x0, 0x0, __SEND_KEY, 0x0, 0x0, 0x0, 0x0, 0x0};
    input->setValue(msg, sizeof(msg));
    input->notify();

    //Key release
    uint8_t msg1[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    input->setValue(msg1, sizeof(msg1));
    input->notify();

    delay(1000);
    changeLedMode(WAITING);
    digitalWrite(LED, LOW);
    setupInterrupts();
  }

  if (ledRestart) {
    Serial.printf("Switching to LED mode %d\n", ledMode);
    delay(50); // make sure loops exit
    ledRestart = false;
    setupInterrupts();
  }

  delay(50);
}

void setupInterrupts() {
  attachInterrupt(digitalPinToInterrupt(__BUTTONPIN_DOWN), buttonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(__BUTTONPIN_UP), buttonReleased, FALLING);
  Serial.println("Setting up interrupts");
}
void clearInterrupts() {
  detachInterrupt(digitalPinToInterrupt(__BUTTONPIN_DOWN));
  detachInterrupt(digitalPinToInterrupt(__BUTTONPIN_UP));
  Serial.println("Clearing interrupts");
}

void changeLedMode(LED_MODES mode) {
    ledMode = mode;
    ledRestart = true;
}

IRAM_ATTR void buttonPressed() {
  clearInterrupts();  // reset in loop
  Serial.println("Detected button press");
  // pressed
  btnFlag = true;
}

IRAM_ATTR void buttonReleased() {
  clearInterrupts();
  Serial.println("Detected button release");
  changeLedMode(ANTICIPATING); //TODO: timeout then switch to WAITING
  Serial.println("Finished buttonReleased");
}
