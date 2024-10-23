#include <SPI.h>
#include <LoRa.h>

// LoRa pins for firebeetle
#define LORA_CS 5     // NSS (CS) pin
#define LORA_RST 14   // RST pin
#define LORA_IRQ 4   // DIO0 pin

// #define LORA_CS 47     // NSS (CS) pin
// #define LORA_RST 21   // RST pin
// #define LORA_IRQ 14   // DIO0 pin



#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(9, 10);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true





int counter = 0;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(5000);
  Serial.println("Adafruit GPS library basic parsing test!");

  Serial.println("LoRa Sender");
  SPI.begin(36, 35, 48, 47);
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
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");



  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  Serial.printIn("GPS Initialized!");


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


void sendLoRaPacket(String data) {
  Serial.print("Sending packet: ");
  Serial.println(data);
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();
  delay(1000);
}