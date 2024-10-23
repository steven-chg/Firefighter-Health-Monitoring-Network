#include <SPI.h>
#include <LoRa.h>

// // LoRa pins for firebeetle
// #define LORA_CS 5     // NSS (CS) pin
// #define LORA_RST 14   // RST pin
// #define LORA_IRQ 4   // DIO0 pin

// LoRa pins for ESP32
#define LORA_CS 47     // NSS (CS) pin
#define LORA_RST 21   // RST pin
#define LORA_IRQ 14   // DIO0 pin

void setup() {
  Serial.begin(115200);
  // for esp32
  SPI.begin(36, 35, 48, 47);
  while (!Serial);
  Serial.println("LoRa Receiver");
  
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  
  Serial.println("Initializing LoRa...");
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa Initialization failed!");
    while (1); // Stop execution here
  }
  
  // Change sync word (0xF3) to match the receiver
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}