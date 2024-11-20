#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>


// TFT Pins
#define TFT_CS 5     // TFT CS pin
#define TFT_RST 10   // TFT RST pin
#define TFT_DC 15    // TFT DC pin

// Touchscreen Pins
#define XP 37        // X+ pin, must be connected to an analog-capable GPIO
#define XM 38        // X- pin, must be connected to a digital GPIO
#define YP 16        // Y+ pin, must be connected to a digital GPIO
#define YM 2        // Y- pin, must be connected to an analog-capable GPIO


// #define XP 11 // X+
// #define XM 6 // X-
// #define YP 7 // Y+
// #define YM 2  // Y-

// TFT Setup (assuming 320x480 HX8357 screen)
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  SPI.begin(36, 35, 48, 47);
  // Initialize TFT
  tft.begin(HX8357D);  // Initialize for HX8357D display type
  tft.setRotation(1);  // Set screen rotation if needed
  tft.fillScreen(HX8357_BLACK);
  tft.setTextColor(HX8357_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("Touch Test");
}

void loop() {
  int touchX, touchY;
  if (readTouch(touchX, touchY)) {
    // Display touch coordinates on TFT
    tft.fillScreen(HX8357_BLACK);  // Clear screen
    tft.setCursor(0, 0);           // Reset cursor to the top left
    tft.print("Touch X: ");
    tft.print(touchX);
    tft.setCursor(0, 30);
    tft.print("Touch Y: ");
    tft.println(touchY);

    // Print to Serial for debugging
    Serial.print("Touch X: ");
    Serial.print(touchX);
    Serial.print(" | Touch Y: ");
    Serial.println(touchY);

    delay(1000);  // Small delay to avoid rapid touch reads
  }
}

// Function to read touch position
bool readTouch(int &x, int &y) {
  // Set pins for X position measurement
  pinMode(XP, INPUT);
  pinMode(XM, INPUT);
  pinMode(YP, OUTPUT);
  pinMode(YM, OUTPUT);

  digitalWrite(YP, HIGH);
  digitalWrite(YM, LOW);

  // Read the X position
  x = analogRead(XP);

  // Set pins for Y position measurement
  pinMode(YP, INPUT);
  pinMode(YM, INPUT);
  pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);

  digitalWrite(XP, HIGH);
  digitalWrite(XM, LOW);

  // Read the Y position
  y = analogRead(YM);

  // Check if values are within a reasonable range
  if (x < 100 || x > 4000) return false;  // No touch detected
  return true;  // Valid touch detected
}

