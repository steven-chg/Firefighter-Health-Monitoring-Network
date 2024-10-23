#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GPS.h>

// LoRa pins for ESP32
// #define LORA_CS 47     // NSS (CS) pin
// #define LORA_RST 21   // RST pin
// #define LORA_IRQ 14   // DIO0 pin

// LoRa pins
#define LORA_CS 5     // NSS (CS) pin
#define LORA_RST 14   // RST pin
#define LORA_IRQ 4   // DIO0 pin

// Use hardware serial port 1 for GPS (ESP32 has multiple hardware UARTs)
#define GPS_RX 16  // Connect to GPS TX
#define GPS_TX 17  // Connect to GPS RX
HardwareSerial gpsSerial(1);  // Use UART1 for GPS
Adafruit_GPS GPS(&gpsSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
#define GPSECHO  true

int counter = 0;

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  while (!Serial);

  // LoRa setup
  Serial.println("LoRa Sender");
  // SPI.begin(36, 35, 48, 47);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  int retries = 10;  // Set a limit for retries
  while (!LoRa.begin(915E6) && retries > 0) {
    Serial.println("Initializing LoRa...");
    delay(500);
    retries--;
  }

  if (retries == 0) {
    Serial.println("LoRa initialization failed.");
    while (1);  // Stop the program if initialization fails
  }
  LoRa.setTxPower(19, PA_OUTPUT_PA_BOOST_PIN);
  // LoRa.setTxPower(23, false);
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initialized!");

  // GPS setup
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // Initialize GPS with pins
  GPS.begin(9600);  // Initialize GPS at 9600 baud
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Output RMC and GGA data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);                // Request updates on antenna status

  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);  // Ask for firmware version

  Serial.println("GPS Initialized!");
}

uint32_t timer = millis();

void loop() {
  // Read data from GPS module
  char c = GPS.read();
  if ((c) && (GPSECHO)) Serial.write(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;  // Fail to parse, so we wait for the next sentence
    }
  }

  // Every 2 seconds, read GPS data and send it over LoRa
  if (millis() - timer > 2000) {
    timer = millis();  // Reset timer

    // If there's a valid GPS fix, send the data
    if (GPS.fix) {
      String gpsData = formatGPSData();  // Get formatted GPS data
      sendLoRaPacket(gpsData);  // Send it via LoRa
    } else{
      String noGPS = "no GPS signal";
      sendLoRaPacket(noGPS);
    }
  }
}

// Function to format GPS data into a string for sending over LoRa
String formatGPSData() {
  String data = "";
  data += "Time: ";
  if (GPS.hour < 10) { data += '0'; }
  data += String(GPS.hour) + ":";
  if (GPS.minute < 10) { data += '0'; }
  data += String(GPS.minute) + ":";
  if (GPS.seconds < 10) { data += '0'; }
  data += String(GPS.seconds) + "." + String(GPS.milliseconds);

  data += " Date: " + String(GPS.day) + "/" + String(GPS.month) + "/20" + String(GPS.year);

  data += " Lat: " + String(GPS.latitude, 4) + GPS.lat;
  data += " Lon: " + String(GPS.longitude, 4) + GPS.lon;

  data += " Speed: " + String(GPS.speed);
  data += " Alt: " + String(GPS.altitude);
  data += " Sat: " + String((int)GPS.satellites);

  return data;  // Return the formatted string
}

// Function to send a LoRa packet
void sendLoRaPacket(String data) {
  Serial.print("Sending packet: ");
  Serial.println(data);

  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();

  delay(1000);  // Wait a second before sending the next packet
}
