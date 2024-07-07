/*

  Modified remote shutter code that is wired to:

  * Neopixels
  * A button (trigger)
  * An external shutter (cheap one-button BT keyboard).

  Keeps cheap shutter awake by clicking its button every 5 minutes if not otherwise used.

  Adapted from:

  Author: Michael Ruck, 9. Juny 2019
  License: GNU Lesser General Public License version 3 (https://opensource.org/licenses/LGPL-3.0)

*/

// https://github.com/madleech/Button
// Using instead of interrupts because it handles debouncing
#include <Button.h>
#include "./settings.h"


//// NEOPIXELS
#include <Adafruit_NeoPixel.h>
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(__NEOPIXEL_COUNT, __NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
enum LED_MODES { OFF, WAITING, SENDING, ANTICIPATING, SUCCESS };
LED_MODES ledMode = OFF;
bool ledRestart = false; // allows for quick mode change, ends loops
Button button(__BUTTONPIN_DOWN);

// on-board LED to indicate bluetooth activity
#define LED 2

bool btnFlag = false;

unsigned long extShutterLastAwakeTime = millis();

void neopixelServer(void*) {
    long timeCurrentModeStarted = millis();

    for(;;) {
        if (ledRestart) {
            delay(50); // make sure loops exit
            Serial.printf("Switching to LED mode %d\n", ledMode);
            timeCurrentModeStarted = millis();
        }
        ledRestart = false;

        switch(ledMode) {
        case OFF:
            strip.clear();
            strip.show();
            delay(50);
            break;
        case WAITING:
            shimmer(255, 0, 0, 1, __NEOPIXEL_MODE_SHIMMER_SPEED);
//             colorWipe(strip.Color(255, 0, 0), 500);
            break;
        case SENDING:
            colorWipe(strip.Color(255, 0, 0), 5); // Red
            ledMode = (ledMode == SENDING) ? OFF: ledMode;
            break;
        case SUCCESS:
            rainbow(20);
            break;
        case ANTICIPATING:
            if ((millis() - timeCurrentModeStarted) > __NEOPIXEL_MODE_ANTICIPATION_TIMEOUT) {
              // we've been anticipating too long, go back to waiting.
              ledMode = WAITING;
            } else {
              theaterChaseRainbow(50);
            }
            break;
        }
        delay(1);
        yield();
    }
}

void setStripToColor(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color); //  Set pixel's color (in RAM)
  }
}

void shimmer(int r, int g, int b, int step, int wait) {
  for (int brightness = __NEOPIXEL_MODE_SHIMMER_MIN_BRIGHTNESS; !ledRestart && brightness < __NEOPIXEL_MODE_SHIMMER_MAX_BRIGHTNESS ; brightness+=step) {
    setStripToColor(strip.Color(r * brightness / 255, g * brightness / 255, b * brightness / 255));
    strip.show();
    delay(wait);
  }
  delay(wait * 10); // may look better
  for (int brightness = __NEOPIXEL_MODE_SHIMMER_MAX_BRIGHTNESS; !ledRestart && brightness > __NEOPIXEL_MODE_SHIMMER_MIN_BRIGHTNESS ; brightness-=step) {
    setStripToColor(strip.Color(r * brightness / 255, g * brightness / 255, b * brightness / 255));
    strip.show();
    delay(wait);
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
  strip.clear();
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

  button.begin();
  button.pressed(); // eat the first keypress if already pressed

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(__EXT_SHUTTER_OUTPUTPIN, OUTPUT);
  digitalWrite(__EXT_SHUTTER_OUTPUTPIN, HIGH);

  // NeoPixel
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();  // Turn OFF all pixels ASAP
  strip.clear();
//   pinMode(__NEOPIXEL_PIN, OUTPUT);
//   digitalWrite(__NEOPIXEL_PIN, LOW);
  changeBackgroundLedMode(WAITING);

  xTaskCreate(neopixelServer, "neopixelServer", 40000, NULL, 10, NULL);
  xTaskCreate(buttonCheckerServer, "buttonCheckerServer", 20000, NULL, 10, NULL);
  xTaskCreate(shutterKeepAwakeServer, "shutterKeepAwakeServer", 10000, NULL, 40, NULL);
}

void clickExtShutter() {
  // click external shutter button
  digitalWrite(__EXT_SHUTTER_OUTPUTPIN, LOW);
  delay(100);
  digitalWrite(__EXT_SHUTTER_OUTPUTPIN, HIGH);

  extShutterLastAwakeTime = millis();
}


void shutterKeepAwakeServer(void*) {
  for(;;) {
    if ((millis() - extShutterLastAwakeTime) >  __EXT_SHUTTER_KEEP_AWAKE_INTERVAL) {
      Serial.println("Waking up external shutter");
      clickExtShutter();
    }
    delay(1000);
  }
}


void buttonCheckerServer(void*) {
  for(;;) {

    if (ledMode == OFF) {
      changeBackgroundLedMode(WAITING);
    }
    if (button.released()) {
      changeBackgroundLedMode(ANTICIPATING);
    } else if (button.pressed()) {
      btnFlag = false;
//        changeBackgroundLedMode(OFF);
      digitalWrite(LED, HIGH);
      Serial.print("Cheese...");
      // show a light show (one pass)
      changeBackgroundLedMode(SENDING);
      Serial.println("...");
      delay(__SEND_DELAY); // delays should match time to display SENDING

      clickExtShutter();

      changeBackgroundLedMode(SUCCESS);
      delay(1000);
      changeBackgroundLedMode(WAITING);
      digitalWrite(LED, LOW);
    }
    delay(100);
  }
}

void loop() {
  delay(1);
}

void changeBackgroundLedMode(LED_MODES mode) {
  if (ledMode != mode) {
    ledMode = mode;
    ledRestart = true;
  }
}
