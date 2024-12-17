#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>

// Buzzer Pin
#define BUZZER_PIN 8

// TFT Pins
#define TFT_CS 5     // TFT CS pin
#define TFT_RST 10   // TFT RST pin
#define TFT_DC 15    // TFT DC pin

// Define LoRa Pins
#define LORA_CS 47   // NSS (CS) pin
#define LORA_RST 21  // RST pin
#define LORA_IRQ 14  // DIO0 pin

// LoRa Network Settings
#define TERMINAL_NODE_ID 1 // The ID of the terminal node where messages stop
#define NODE_ID 1          // Unique ID for this node

// Maximum number of messages to track
#define MAX_TRACKED_MESSAGES 50

// Battery Variables
const int VBAT_SENSE_PIN = 4;  // GPIO pin 4 (ADC1_CH3) for battery sensing
const float DIVIDER_RATIO = 12.0;  // Voltage divider ratio (100k + 10k) / 10k = 11
const float ADC_REFERENCE = 3.3;  // ESP32 ADC reference voltage
const float ADC_RESOLUTION = 4095.0;  // ESP32 has 12-bit ADC (2^12 - 1)
                        

float battery1;
float battery2;
long batteryCentral;
int timeInterval;
int batteryAlert1;
int batteryAlert2;
int batteryCentralAlert;
int battery1Time;
int battery2Time;
int last_update;
int update;

// TFT Variables
int infoWidth;
int infoHeight;
int boxWidth;
int boxHeight;
int infoBox_x;
int infoBox_y;
int node2Box_x;
int node2Box_y;
int node3Box_x;
int node3Box_y;
int row_height;

// Row of Text
int gps_row;
int temp_row;
int heart_row;
int accel_row;
int hop_row;
int signal_row;
int image_row;
int battery_row;

// Text Cursor
int gpsCursor1;
int tempCursor1;
int heartCursor1;
int accelCursor1;
int hopCursor1;
int signalCursor1;
int batteryCursor1;

int gpsCursor2;
int tempCursor2;
int heartCursor2;
int accelCursor2;
int hopCursor2;
int signalCursor2;
int batteryCursor2;

int lastReceivedNode2;
int lastReceivedNode3;

bool heartState=true;

// TFT Setup (assuming 320x480 HX8357 screen)
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

// Array to store recent message IDs
unsigned long trackedMsgIDs[MAX_TRACKED_MESSAGES];
int trackedIndex = 0;

const uint8_t battery100[] = {
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 1
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 2 (top of the battery body)
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 3
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 4 (battery charge)
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 5
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 6
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 7
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 8
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 9
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 10 (battery charge)
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 11
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 12
    0b11101111, 0b10111110, 0b11111011, 0b11101111, // Row 13
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 14 (bottom of the battery body)
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 15
    0b11111111, 0b11111111, 0b11111111, 0b11111100  // Row 16
};

const uint8_t battery75[] = {
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 1
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 2 (top of the battery body)
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 3
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 4 (battery charge)
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 5
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 6
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 7
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 8
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 9
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 10 (battery charge)
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 11
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 12
    0b11101111, 0b10111110, 0b11111000, 0b00001111, // Row 13
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 14 (bottom of the battery body)
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 15
    0b11111111, 0b11111111, 0b11111111, 0b11111100  // Row 16
};

const uint8_t battery50[] = {
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 1
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 2 (top of the battery body)
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 3
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 4 (battery charge)
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 5
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 6
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 7
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 8
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 9
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 10 (battery charge)
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 11
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 12
    0b11101111, 0b10111110, 0b00000000, 0b00001111, // Row 13
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 14 (bottom of the battery body)
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 15
    0b11111111, 0b11111111, 0b11111111, 0b11111100  // Row 16
};

const uint8_t battery25[] = {
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 1
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 2 (top of the battery body)
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 3
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 4 (battery charge)
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 5
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 6
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 7
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 8
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 9
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 10 (battery charge)
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 11
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 12
    0b11101111, 0b10000000, 0b00000000, 0b00001111, // Row 13
    0b11100000, 0b00000000, 0b00000000, 0b00001111, // Row 14 (bottom of the battery body)
    0b11111111, 0b11111111, 0b11111111, 0b11111100, // Row 15
    0b11111111, 0b11111111, 0b11111111, 0b11111100  // Row 16
};

const uint8_t smallHeart[] = {
  0b00001111,0b10000001,0b11110000,
  0b00001111,0b10000001,0b11110000,
  0b00111111,0b11111111,0b11111100,
  0b00111111,0b11111111,0b11111100,
  0b11110000,0b11111111,0b11111111,
  0b11110000,0b11111111,0b11111111,
  0b11110111,0b11111111,0b11111111,
  0b11110111,0b11111111,0b11111111,
  0b11111111,0b11111111,0b11111111,
  0b11111111,0b11111111,0b11111111,
  0b11110111,0b11111111,0b11111111,
  0b11110111,0b11111111,0b11111111,
  0b00111111,0b11111111,0b11111100,
  0b00111111,0b11111111,0b11111100,
  0b00001111,0b11111111,0b11110000,
  0b00000111,0b11111111,0b11100000,
  0b00000111,0b11111111,0b11100000,
  0b00000111,0b11111111,0b11100000,
  0b00000000,0b11111111,0b00000000,
  0b00000000,0b11111111,0b00000000,
  0b00000000,0b01111110,0b00000000,
  0b00000000,0b01111110,0b00000000,
  0b00000000,0b00011000,0b00000000,
  0b00000000,0b00000000,0b00000000
};
const uint8_t bigHeart[] = {
  0b00000111,0b11110000,0b00001111,0b11100000,
  0b00000111,0b11110000,0b00001111,0b11100000,
  0b00000111,0b11110000,0b00001111,0b11100000,
  0b00011111,0b11111110,0b01111111,0b11111000,
  0b00011111,0b11111110,0b01111111,0b11111000,
  0b11111000,0b00111111,0b11111111,0b11111111,
  0b11111000,0b00111111,0b11111111,0b11111111,
  0b11111000,0b00111111,0b11111111,0b11111111,
  0b11111001,0b11111111,0b11111111,0b11111111,
  0b11111001,0b11111111,0b11111111,0b11111111,
  0b11111001,0b11111111,0b11111111,0b11111111,
  0b11111111,0b11111111,0b11111111,0b11111111,
  0b11111111,0b11111111,0b11111111,0b11111111,
  0b11111001,0b11111111,0b11111111,0b11111111,
  0b11111001,0b11111111,0b11111111,0b11111111,
  0b11111001,0b11111111,0b11111111,0b11111111,
  0b00011111,0b11111111,0b11111111,0b11111000,
  0b00011111,0b11111111,0b11111111,0b11111000,
  0b00011111,0b11111111,0b11111111,0b11111000,
  0b00000111,0b11111111,0b11111111,0b11100000,
  0b00000111,0b11111111,0b11111111,0b11100000,
  0b00000001,0b11111111,0b11111111,0b10000000,
  0b00000001,0b11111111,0b11111111,0b10000000,
  0b00000001,0b11111111,0b11111111,0b10000000,
  0b00000000,0b00111111,0b11111100,0b00000000,
  0b00000000,0b00111111,0b11111100,0b00000000,
  0b00000000,0b00111111,0b11111100,0b00000000,
  0b00000000,0b00001111,0b11110000,0b00000000,
  0b00000000,0b00001111,0b11110000,0b00000000,
  0b00000000,0b00000001,0b10000000,0b00000000,
  0b00000000,0b00000001,0b10000000,0b00000000,
  0b00000000,0b00000001,0b10000000,0b00000000
};

const uint8_t signalStrength[] = {
    0b00000000,0b00000001,0b10000000,0b00000000,
    0b00000000,0b00000001,0b10000000,0b00000000,
    0b00000000,0b00000111,0b11100000,0b00000000,
    0b00000000,0b00000111,0b11100000,0b00000000,
    0b00000000,0b00011111,0b11111000,0b00000000,
    0b00000000,0b00011111,0b11111000,0b00000000,
    0b00000000,0b01111111,0b11111110,0b00000000,
    0b00000000,0b01111111,0b11111110,0b00000000,
    0b00000001,0b11111111,0b11111111,0b10000000,
    0b00000001,0b11111111,0b11111111,0b10000000,
    0b00000000,0b00000001,0b10000000,0b00000000,
    0b00000000,0b00000001,0b10000000,0b00000000,
    0b00000000,0b00000001,0b10000000,0b00000000,
    0b00000000,0b00000001,0b10000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000
};
// Location pin symbol (32x32 pixels, 4 bytes per row for 32 rows)
const uint8_t locationPin[] = {
    0b00000000,0b00111111,0b11110000,0b00000000,
    0b00000001,0b11111111,0b11111100,0b00000000,
    0b00000111,0b11111111,0b11111111,0b00000000,
    0b00001111,0b11111111,0b11111111,0b10000000,
    0b00011111,0b11111111,0b11111111,0b11000000,
    0b00011111,0b11000000,0b00111111,0b11000000,
    0b00111111,0b10000000,0b00011111,0b11100000,
    0b00111111,0b10000000,0b00011111,0b11100000,
    0b00111111,0b10000000,0b00011111,0b11100000,
    0b00111111,0b10000000,0b00011111,0b11100000,
    0b00111111,0b10000000,0b00011111,0b11100000,
    0b00011111,0b11000000,0b00111111,0b11000000,
    0b00011111,0b11111111,0b11111111,0b11000000,
    0b00001111,0b11111111,0b11111111,0b10000000,
    0b00000111,0b11111111,0b11111111,0b00000000,
    0b00000011,0b11111111,0b11111110,0b00000000,
    0b00000001,0b11111111,0b11111100,0b00000000,
    0b00000000,0b11111111,0b11111000,0b00000000,
    0b00000000,0b01111111,0b11110000,0b00000000,
    0b00000000,0b00111111,0b11100000,0b00000000,
    0b00000000,0b00011111,0b11000000,0b00000000,
    0b00000000,0b00001111,0b10000000,0b00000000,
    0b00000000,0b00000111,0b00000000,0b00000000,
    0b00000000,0b00000010,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000,
    0b00000000,0b00000000,0b00000000,0b00000000
};


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // while (!Serial);
  timeInterval = 50;
  batteryCentral = 0;
  battery1 = 4.2;
  battery2 = 4.2;
  battery1Time = 0;
  battery2Time = 0;
  batteryAlert1 = 0;
  batteryAlert2 = 0;
  batteryCentralAlert = 0;
  last_update = 0;
  update = 0;
  // Configure ADC
  analogReadResolution(12);  // Set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation for 3.3V full-scale range

  // Initialize LoRa
  SPI.begin(36, 35, 48, 47);  // Set SPI pins for LoRa

  // Buzzer Setup
  pinMode(BUZZER_PIN, OUTPUT); // configure GPIO8 as an output (can be set to high (buzzer on) or low (buzzer off)

  // Initialize TFT
  delay(3000);
  tft.begin(HX8357D);  // Initialize for HX8357D display type
  tft.setRotation(1);  // Set screen rotation if needed
  infoWidth = tft.width()-10;
  infoHeight = 20;
  boxWidth = (tft.width()-15)/2;
  boxHeight = (tft.height()-15-infoHeight);
  infoBox_x = 5;
  infoBox_y = 5;
  node2Box_x = 5;
  node2Box_y = 10+infoHeight;
  node3Box_x = 10+boxWidth;
  node3Box_y = 10+infoHeight;
  row_height = 30;
  image_row = node2Box_y + 10;
  gps_row = node2Box_y+50;
  temp_row = gps_row + row_height*2;
  heart_row = temp_row + row_height;
  accel_row = heart_row + row_height;
  hop_row = accel_row + row_height;
  signal_row = hop_row + row_height;
  battery_row = signal_row + row_height;

  gpsCursor1 = node2Box_x+5+72;
  tempCursor1 = node2Box_x+5+65;
  heartCursor1 = node2Box_x+5+125;
  accelCursor1 = node2Box_x+5+75;
  hopCursor1 = node2Box_x+5+110;
  signalCursor1 = node2Box_x+5+90;
  batteryCursor1 = node2Box_x+5+145;

  gpsCursor2 = node3Box_x+5+72;
  tempCursor2 = node3Box_x+5+65;
  heartCursor2 = node3Box_x+5+125;
  accelCursor2 = node3Box_x+5+75;
  hopCursor2 = node3Box_x+5+110;
  signalCursor2 = node3Box_x+5+90;
  batteryCursor2 = node3Box_x+5+145;

  tft.setTextColor(HX8357_WHITE);
  tft.fillScreen(HX8357_WHITE);
  tft.setTextSize(2);

  // Info Screen
  tft.fillRect(infoBox_x, infoBox_y, infoWidth, infoHeight, HX8357_RED);
  tft.setCursor(infoBox_x+5, infoBox_y+2);
  tft.print("FireFighter Monitoring Network");
  tft.setCursor(tft.width()-70, infoBox_y+2);
  tft.fillRect(tft.width()-70, infoBox_y+2, 65, 16, HX8357_RED);
  tft.print("4.2V");
  tft.fillRect(tft.width()-105, infoBox_y+2, 32, 16, HX8357_RED);
  tft.setTextColor(HX8357_WHITE);
  tft.drawBitmap(tft.width()-105, infoBox_y+2, battery100, 32, 16, HX8357_WHITE);

  // Node 2 Box
  tft.fillRect(node2Box_x, node2Box_y, boxWidth, boxHeight, HX8357_BLACK);
  tft.setCursor(node2Box_x+5, node2Box_y+10);
  tft.setTextSize(3);
  tft.print("FireMan 1");
  tft.setTextSize(2);
  tft.setCursor(node2Box_x+5, gps_row);
  tft.print("GPS  :");
  tft.drawBitmap(node2Box_x+40, gps_row-5, locationPin, 32, 32, HX8357_YELLOW);
  tft.setCursor(node2Box_x+5, temp_row);
  tft.print("Temp:");
  tft.setCursor(node2Box_x+5, heart_row);
  tft.print("HeartRate:");
  tft.setCursor(node2Box_x+5, accel_row);
  tft.print("Accel:");
  tft.setCursor(node2Box_x+5, hop_row);
  tft.print("Hop&Time:");
  tft.setCursor(node2Box_x+5, signal_row);
  tft.drawBitmap(node2Box_x+50, signal_row, signalStrength, 32, 32, HX8357_MAGENTA);
  tft.print("RSSI  :");
  tft.setCursor(node2Box_x+5, battery_row);
  tft.drawBitmap(node2Box_x+100, battery_row, battery100, 32, 16, HX8357_WHITE);
  tft.print("Battery    :");
  tft.setCursor(batteryCursor1, battery_row);
  tft.print("4.2V");

  // Node 3 Box
  tft.fillRect(node3Box_x, node3Box_y, boxWidth, boxHeight, HX8357_BLACK);
  tft.setCursor(node3Box_x+5, node3Box_y+10);
  tft.setTextSize(3);
  tft.print("FireMan 2");
  tft.setTextSize(2);
  tft.setCursor(node3Box_x+5, gps_row);
  tft.print("GPS  :");
  tft.drawBitmap(node3Box_x+40, gps_row-5, locationPin, 32, 32, HX8357_YELLOW);
  tft.setCursor(node3Box_x+5, temp_row);
  tft.print("Temp:");
  tft.setCursor(node3Box_x+5, heart_row);
  tft.print("HeartRate:");
  tft.setCursor(node3Box_x+5, accel_row);
  tft.print("Accel:");
  tft.setCursor(node3Box_x+5, hop_row);
  tft.print("Hop&Time:");
  tft.setCursor(node3Box_x+5, signal_row);
  tft.drawBitmap(node3Box_x+50, signal_row, signalStrength, 32, 32, HX8357_MAGENTA);
  tft.print("RSSI  :");
  tft.setCursor(node3Box_x+5, battery_row);
  tft.drawBitmap(node3Box_x+100, battery_row, battery100, 32, 16, HX8357_WHITE);
  tft.print("Battery    :");
  tft.setCursor(batteryCursor2, battery_row);
  tft.print("4.2V");

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); // Set LoRa pins
  if (!LoRa.begin(915E6)) {    // Set frequency for North America
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  LoRa.setSyncWord(0xF3);
  Serial.print("Node ");
  Serial.print(NODE_ID);
  Serial.println(" initialized");
  lastReceivedNode2 = millis();
  lastReceivedNode3 = millis();
}

void readBatteryPercentage() {
  
  // Convert ADC reading to actual battery voltage
  float averageReading = batteryCentral/timeInterval;
  float batteryVoltage = (averageReading / ADC_RESOLUTION) * ADC_REFERENCE * DIVIDER_RATIO;
  float percentage = (batteryVoltage-3)/1.2*100;
  batteryCentral = 0;

  if(percentage<=10){
    batteryCentralAlert=1;
  }
  else{
    batteryCentralAlert=0;
  }

  tft.setCursor(tft.width()-70, infoBox_y+2);
  tft.fillRect(tft.width()-70, infoBox_y+2, 65, 16, HX8357_RED);
  tft.setTextSize(2);
  if(batteryCentralAlert==1)tft.setTextColor(HX8357_YELLOW);
  else tft.setTextColor(HX8357_WHITE);
  tft.print(batteryVoltage);
  tft.print("V");
  tft.fillRect(tft.width()-105, infoBox_y+2, 32, 16, HX8357_RED);
  tft.setTextColor(HX8357_WHITE);
  
  if(percentage<=25){
    if(batteryCentralAlert==1)tft.drawBitmap(tft.width()-105, infoBox_y+2, battery25, 32, 16, HX8357_YELLOW);
    else tft.drawBitmap(tft.width()-105, infoBox_y+2, battery25, 32, 16, HX8357_WHITE);
  }
  else if(percentage<=50){
    tft.drawBitmap(tft.width()-105, infoBox_y+2, battery50, 32, 16, HX8357_WHITE);
  }
  else if(percentage<=75){
    tft.drawBitmap(tft.width()-105, infoBox_y+2, battery75, 32, 16, HX8357_WHITE);
  }
  else{
    tft.drawBitmap(tft.width()-105, infoBox_y+2, battery100, 32, 16, HX8357_WHITE);
  }
  
  tft.fillRect(batteryCursor1, battery_row, 50, 20, HX8357_BLACK);
  tft.setCursor(batteryCursor1, battery_row);
  if(batteryAlert1)tft.setTextColor(HX8357_RED);
  percentage = (battery1-3)/1.2*100;

  tft.fillRect(batteryCursor1, battery_row, 50, 20, HX8357_BLACK);
  tft.setCursor(batteryCursor1, battery_row);
  if(batteryAlert1)tft.setTextColor(HX8357_RED);
  tft.print(battery1);
  tft.print("V");
  tft.setTextColor(HX8357_WHITE);

  tft.fillRect(node2Box_x+100, battery_row, 32,16, HX8357_BLACK);
  if(percentage<=25){
    if(batteryAlert1==1)tft.drawBitmap(node2Box_x+100, battery_row, battery25, 32, 16, HX8357_RED);
    else tft.drawBitmap(node2Box_x+100, battery_row, battery25, 32, 16, HX8357_WHITE);
  }
  else if(percentage<=50){
    tft.drawBitmap(node2Box_x+100, battery_row, battery50, 32, 16, HX8357_WHITE);
  }
  else if(percentage<=75){
    tft.drawBitmap(node2Box_x+100, battery_row, battery75, 32, 16, HX8357_WHITE);
  }
  else{
    tft.drawBitmap(node2Box_x+100, battery_row, battery100, 32, 16, HX8357_WHITE);
  }
  percentage = (battery2-3)/1.2*100;

  tft.fillRect(batteryCursor2, battery_row, 50, 20, HX8357_BLACK);
  tft.setCursor(batteryCursor2, battery_row);
  if(batteryAlert2)tft.setTextColor(HX8357_RED);
  tft.print(battery2);
  tft.print("V");
  tft.setTextColor(HX8357_WHITE);
  
  tft.fillRect(node3Box_x+100, battery_row, 32,16, HX8357_BLACK);
  if(percentage<=25){
    if(batteryAlert2==1)tft.drawBitmap(node3Box_x+100, battery_row, battery25, 32, 16, HX8357_RED);
    else tft.drawBitmap(node3Box_x+100, battery_row, battery25, 32, 16, HX8357_WHITE);
  }
  else if(percentage<=50){
    tft.drawBitmap(node3Box_x+100, battery_row, battery50, 32, 16, HX8357_WHITE);
  }
  else if(percentage<=75){
    tft.drawBitmap(node3Box_x+100, battery_row, battery75, 32, 16, HX8357_WHITE);
  }
  else{
    tft.drawBitmap(node3Box_x+100, battery_row, battery100, 32, 16, HX8357_WHITE);
  }
  Serial.print("Variable_1:");
  Serial.print(battery1);
  Serial.print(",Variable_2:");
  Serial.println(battery2);
}

// Add a message ID to the tracking array
void trackMessage(unsigned long msgID) {
  trackedMsgIDs[trackedIndex] = msgID;
  trackedIndex = (trackedIndex + 1) % MAX_TRACKED_MESSAGES;
}

// Check if a message ID has already been processed
bool isDuplicate(unsigned long msgID) {
  for (int i = 0; i < MAX_TRACKED_MESSAGES; i++) {
    if (trackedMsgIDs[i] == msgID) {
      return true;
    }
  }
  return false;
}

bool isValidString(const String &input) {
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    // Check if the character is a digit, letter, or one of the allowed symbols
    if (!(isDigit(c) || isAlpha(c) || c == '+' || c == '-' || c == ',' || c == '.' || ':' || ';')) {
      return false; // If not, the string is invalid
    }
  }
  return true;
}

void playTone(int frequency, int duration) {
  unsigned long period = 1000000 / frequency;
  unsigned long endTime = millis() + duration;
  
  while(millis() < endTime) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(period / 2);
  }
}

void playUrgentBeeps() {
  //playTone(4000,30);
}

void heartAnimation(){
  int time = millis();
  if(time-lastReceivedNode2<=5000 && time-lastReceivedNode3<=5000){
    if(heartState){
      tft.drawBitmap(node2Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
      tft.drawBitmap(node3Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
      heartState = !heartState;
    }
    else{
      tft.fillRect(node2Box_x+190, image_row, 32, 32, HX8357_BLACK);
      tft.fillRect(node3Box_x+190, image_row, 32, 32, HX8357_BLACK);
      tft.drawBitmap(node2Box_x+194, image_row+4, smallHeart, 24, 24, HX8357_RED);
      tft.drawBitmap(node3Box_x+194, image_row+4, smallHeart, 24, 24, HX8357_RED);
      heartState = !heartState;
    }
  }
  else if(time-lastReceivedNode2<=5000){
    if(heartState){
      tft.drawBitmap(node2Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
      heartState = !heartState;
    }
    else{
      tft.fillRect(node2Box_x+190, image_row, 32, 32, HX8357_BLACK);
      tft.drawBitmap(node2Box_x+194, image_row+4, smallHeart, 24, 24, HX8357_RED);
      heartState = !heartState;
    }
    tft.drawBitmap(node3Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
    battery2 = 0;
  }
  else if(time-lastReceivedNode3<=5000){
    if(heartState){
      tft.drawBitmap(node3Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
      heartState = !heartState;
    }
    else{
      tft.fillRect(node3Box_x+190, image_row, 32, 32, HX8357_BLACK);
      tft.drawBitmap(node3Box_x+194, image_row+4, smallHeart, 24, 24, HX8357_RED);
      heartState = !heartState;
    }
    tft.drawBitmap(node2Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
    battery1 = 0;
  }
  else{
    tft.drawBitmap(node2Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
    tft.drawBitmap(node3Box_x+190, image_row, bigHeart, 32, 32, HX8357_RED);
    battery1 = 0;
    battery2 = 0;
  }
}

void loop() {
  heartAnimation();
  batteryCentral+=analogRead(VBAT_SENSE_PIN);
  if(batteryCentralAlert)playUrgentBeeps();
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read the sender ID
    int senderID = LoRa.read();
    int numHop = LoRa.read();
    if(numHop<3){
      int RSSI = LoRa.packetRssi();
      // Read the message ID (4 bytes)
      unsigned long msgID = 0;
      msgID |= (unsigned long)LoRa.read() << 24;
      msgID |= (unsigned long)LoRa.read() << 16;
      msgID |= (unsigned long)LoRa.read() << 8;
      msgID |= (unsigned long)LoRa.read();

      // Read the Data
      int tempAlert = LoRa.read();
      int motionAlert = LoRa.read();
      int HRAlert = LoRa.read();
      int batteryAlert = LoRa.read(); 
      int gpsReading = LoRa.read();
      String data = LoRa.readString();
      
      String temperature;
      String acceleration;
      String heartRate;
      float battery = 0;
      String lat;
      String lon;
      
      bool noise = false;

      for(int i=0;i<6;i++){
        // Find the semicolon (end of key-value pair)
        int separatorIndex = data.indexOf(';');
        if(separatorIndex==-1){
          noise = true;
          break;
        }
        // Extract the current key-value pair
        if(i==0){
          String value = data.substring(0, min(8,separatorIndex));
          if(isValidString(value))temperature = value;
          else{
            noise = true;
            break;
          }
        }
        else if(i==1){
          String value = data.substring(0, min(5,separatorIndex));
          if(isValidString(value))acceleration = value;
          else{
            noise = true;
            break;
          }
        }
        else if(i==2){
          String value = data.substring(0, min(8,separatorIndex));
          if(isValidString(value))heartRate = value;
          else{
            noise = true;
            break;
          }
        }
        else if(i==3){
          String value = data.substring(0, min(4,separatorIndex));
          if(isValidString(value))battery = value[0]-'0' + (value[2]-'0')*0.1 + (value[3]-'0')*0.01;
          else{
            noise = true;
            break;
          }
        }
        else if(i==4){
          String value = data.substring(0, min(12,separatorIndex));
          if(isValidString(value))lat = value;
          else{
            noise = true;
            break;
          }
        }
        else{
          String value = data.substring(0, min(12,separatorIndex));
          if(isValidString(value))lon = value;
          else{
            noise = true;
            break;
          }
        }
        // Remove the processed part from the original string
        data = data.substring(separatorIndex + 1);
      }
      if(!noise && RSSI>-110){
        // Check for duplicates
        if (isDuplicate(msgID)) {
          // Serial.println("Duplicate message. Ignoring.");
          return;
        }

        // Track the new message ID
        trackMessage(msgID);
        // Serial.println("Message has reached the terminal node!!!!!!");
        // Serial.print("Final Message: ");
        // Serial.print("Received Message ID: ");
        // Serial.print(msgID);
        // Serial.print(" from Node ");
        // Serial.print(senderID);
        // Serial.print(" with # of hop =  ");
        // Serial.print(numHop);
        // Serial.print("' with RSSI ");
        // Serial.println(RSSI);
        // Serial.print("' with temp ");
        // Serial.println(temperature);
        // Serial.print("' with GPS ");
        // Serial.print(lat);
        // Serial.print("' ");
        // Serial.println(lon);
        // Serial.print("' with Battery ");
        // Serial.println(battery);
        // Serial.print("' with acceleration ");
        // Serial.println(acceleration);
        // Serial.print("' with heartRate ");
        // Serial.println(heartRate);
        // Serial.print("' with tempAlert ");
        // Serial.println(tempAlert);
        // Serial.print("' with motionAlert ");
        // Serial.println(motionAlert);
        // Serial.print("' with HRAlert ");
        // Serial.println(HRAlert);
        // Serial.print("' with batteryAlert ");
        // Serial.println(batteryAlert);
        if(senderID==2){
          batteryAlert1 = batteryAlert;
          battery1 = battery;
          int time = millis();
          int deliveredTime = time-lastReceivedNode2;
          lastReceivedNode2 = time;
          tft.fillRect(gpsCursor1, gps_row, node3Box_x-5-gpsCursor1, 20+row_height, HX8357_BLACK);
          tft.setCursor(gpsCursor1, gps_row);
          if(gpsReading){
            tft.print(lat);
            tft.setCursor(gpsCursor1, gps_row+row_height);
            tft.print(lon);
          }
          else{
            tft.setTextColor(HX8357_RED);
            tft.print("No Signal!");
          }
          if(tempAlert){
            tft.setTextColor(HX8357_RED);
          }
          else{
            tft.setTextColor(HX8357_WHITE);
          }
          tft.fillRect(tempCursor1, temp_row, node3Box_x-5-tempCursor1, 20, HX8357_BLACK);
          tft.setCursor(tempCursor1, temp_row);
          tft.print(temperature);
          tft.print(" C");
          tft.setTextColor(HX8357_WHITE);
          if(HRAlert){
            tft.setTextColor(HX8357_RED);
          }
          tft.fillRect(heartCursor1, heart_row, node3Box_x-5-heartCursor1, 20, HX8357_BLACK);
          tft.setCursor(heartCursor1, heart_row);
          tft.print(heartRate);
          tft.print(" bpm");
          tft.setTextColor(HX8357_WHITE);
          if(motionAlert){
            tft.setTextColor(HX8357_RED);
          }
          tft.fillRect(accelCursor1, accel_row, node3Box_x-5-accelCursor1, 20, HX8357_BLACK);
          tft.setCursor(accelCursor1, accel_row);
          tft.print(acceleration);
          tft.print(" m/s^2");
          tft.setTextColor(HX8357_WHITE);
          tft.fillRect(hopCursor1, hop_row, node3Box_x-5-hopCursor1, 20, HX8357_BLACK);
          tft.setCursor(hopCursor1, hop_row);
          tft.print(numHop);
          tft.print("& ");
          tft.print(deliveredTime);
          tft.print("ms");
          tft.fillRect(signalCursor1, signal_row, node3Box_x-5-signalCursor1, 20, HX8357_BLACK);
          tft.setCursor(signalCursor1, signal_row);
          tft.print(RSSI);
          tft.print(" dBm");
        }
        else{
          batteryAlert2 = batteryAlert;
          battery2 = battery;
          int time = millis();
          int deliveredTime = time-lastReceivedNode3;
          lastReceivedNode3 = time;
          tft.fillRect(gpsCursor2, gps_row, tft.width()-5-gpsCursor2, 20+row_height, HX8357_BLACK);
          tft.setCursor(gpsCursor2, gps_row);
          if(gpsReading){
            tft.print(lat);
            tft.setCursor(gpsCursor2, gps_row+row_height);
            tft.print(lon);
          }
          else{
            tft.setTextColor(HX8357_RED);
            tft.print("No Signal!");
          }
          if(tempAlert){
            tft.setTextColor(HX8357_RED);
          }
          else{
            tft.setTextColor(HX8357_WHITE);
          }
          tft.fillRect(tempCursor2, temp_row, tft.width()-6-tempCursor2, 20, HX8357_BLACK);
          tft.setCursor(tempCursor2, temp_row);
          tft.print(temperature);
          tft.print(" C");
          tft.setTextColor(HX8357_WHITE);
          if(HRAlert){
            tft.setTextColor(HX8357_RED);
          }
          tft.fillRect(heartCursor2, heart_row, tft.width()-6-heartCursor2, 20, HX8357_BLACK);
          tft.setCursor(heartCursor2, heart_row);
          tft.print(heartRate);
          tft.print(" bpm");
          tft.setTextColor(HX8357_WHITE);
          if(motionAlert){
            tft.setTextColor(HX8357_RED);
          }
          tft.fillRect(accelCursor2, accel_row, tft.width()-6-accelCursor2, 20, HX8357_BLACK);
          tft.setCursor(accelCursor2, accel_row);
          tft.print(acceleration);
          tft.print(" m/s^2");
          tft.setTextColor(HX8357_WHITE);
          tft.fillRect(hopCursor2, hop_row, tft.width()-6-hopCursor2, 20, HX8357_BLACK);
          tft.setCursor(hopCursor2, hop_row);
          tft.print(numHop);
          tft.print("& ");
          tft.print(deliveredTime);
          tft.print("ms");
          tft.fillRect(signalCursor2, signal_row, tft.width()-6-signalCursor2, 20, HX8357_BLACK);
          tft.setCursor(signalCursor2, signal_row);
          tft.print(RSSI);
          tft.print(" dBm");
        }
        if(tempAlert|motionAlert|HRAlert|batteryAlert) playUrgentBeeps();
      }
    }
  }
  if(update==timeInterval){
    readBatteryPercentage();
    update=0;
  }
  update+=1;
  delay(100); // Small delay to avoid spamming
}
