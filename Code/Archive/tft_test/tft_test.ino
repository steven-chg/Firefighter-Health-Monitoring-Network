#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_HX8357.h>

// LoRa Pins
#define LORA_CS 47   // NSS (CS) pin
#define LORA_RST 21  // RST pin
#define LORA_IRQ 14  // DIO0 pin

// TFT Pins
#define TFT_CS 5     // TFT CS pin
#define TFT_RST 10    // TFT RST pin
#define TFT_DC 15     // TFT DC pin

// TFT Setup (assuming 320x480 HX8357 screen)
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  SPI.begin(36, 35, 48, 47);
  while (!Serial);

  // Initialize TFT
  tft.begin(HX8357D);  // Initialize for HX8357D display type
  tft.setRotation(1);  // Set screen rotation if needed
  tft.fillScreen(HX8357_BLACK);
  tft.setTextColor(HX8357_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.print("Initializing...");

  // Initialize LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); // Set LoRa pins
  if (!LoRa.begin(915E6)) {                  // Frequency for North America
    Serial.println("LoRa initialization failed!");
    tft.println("LoRa init failed!");
    while (1);
  }
  LoRa.setSyncWord(0xF3); // Set sync word to match receiver
  Serial.println("LoRa initialized successfully!");
  tft.println("LoRa initialized");
}

void loop() {
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("Received packet:");
    String LoRaData = "";

    // Read incoming packet
    while (LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }

    // Display received packet on Serial and TFT
    Serial.println(LoRaData);
    tft.fillScreen(HX8357_BLACK);  // Clear screen
    tft.setCursor(0, 0);           // Reset cursor to the top left
    tft.print("Received:");
    tft.setCursor(0, 30);
    tft.println(LoRaData);

    // Display RSSI on TFT
    int rssi = LoRa.packetRssi();
    tft.setCursor(0, 60);
    tft.print("RSSI: ");
    tft.println(rssi);
  }
}
