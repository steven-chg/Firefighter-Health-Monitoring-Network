#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_GPS.h>

Adafruit_BMP085 bmp;
Adafruit_LSM6DSO32 dso32;

// Define LoRa Pins
#define LORA_CS 47   // NSS (CS) pin
#define LORA_RST 21  // RST pin
#define LORA_IRQ 14  // DIO0 pin

// LoRa Network Settings
#define TERMINAL_NODE_ID 1 // The ID of the terminal node where messages stop
#define NODE_ID 3          // Unique ID for this node

// Maximum number of messages to track
#define MAX_TRACKED_MESSAGES 50

#define SDA_PIN 37
#define SCL_PIN 38
TwoWire customWire = TwoWire(0); // Create a custom I2C instance on port 0

#define LO_PLUS_PIN 9
#define LO_MINUS_PIN 10
#define BUZZER_PIN 8
#define LED_BUILTIN 18

// Use hardware serial port 1 for GPS (ESP32 has multiple hardware UARTs)
#define GPS_RX 16  // Connect to GPS TX
#define GPS_TX 17  // Connect to GPS RX
HardwareSerial gpsSerial(1);  // Use UART1 for GPS
Adafruit_GPS GPS(&gpsSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
#define GPSECHO  true

int counter = 0;

// Array to store recent message IDs
unsigned long trackedMsgIDs[MAX_TRACKED_MESSAGES];
int trackedIndex = 0;

// Alert thesholds
const int HR_MAX = 150;
const int HR_MIN = 0;              /////////////////////////////////////////// CHANGE BACK TO 40
const float TEMP_MAX = 27.0;        /////////////////////////////////////////// CHANGE BACK TO 40
const float MOVEMENT_THRESHOLD = 0.5;
const float MS2_TO_G = 1.0 / 9.81;

// Alert timing thresholds
const unsigned long HR_ALERT_TIME = 30000;  // 30 seconds
const unsigned long TEMP_ALERT_TIME = 60000; // 3 minutes          /////////////// CHANGE BACK TO 3 minutes 180000
const unsigned long MOTION_ALERT_TIME = 60000;  // 60 seconds

// GPS update frequency
const unsigned long NORMAL_UPDATE_FREQ = 1000; // 1 second
const unsigned long EMERGENCY_UPDATE_FREQ = 500; // half a second

struct AlertStatus{
  bool isHRAlert = false; // check if we should generate alert currently for heartrate
  bool isTempAlert = false; // check if we should generate alert currently for temperature
  bool isMotionAlert = false; // check if we should generate alert currently for no motion
  unsigned long hrAlertStart = 0; // when did heartrate start violating threshold
  unsigned long tempAlertStart = 0; // when did temperature start exceeding threshold
  float accX = 0.0; // last relevant x acc
  float accY = 0.0; // last relevant y acc
  float accZ = 0.0; // last relevant z acc 
  unsigned long motionAlertStart = 0; // when did motion start becoming stagnant
  unsigned long lastUpdate = 0; // track last time update was sent to central
  bool inEmergency = false; // bool of whether current node is in emergency status
};

// Structure to hold message details
struct LoRaMessage {
  int senderID;
  unsigned long msgID;
  String payload;
};

AlertStatus wearableAlert;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  // Initialize LoRa
  SPI.begin(36, 35, 48, 47);  // Set SPI pins for LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); // Set LoRa pins
  if (!LoRa.begin(915E6)) {    // Set frequency for North America
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  
  Serial.print("Node ");
  Serial.print(NODE_ID);
  Serial.println(" initialized");

  Serial.println("Sensors Test!");
  customWire.begin(SDA_PIN, SCL_PIN);

  // Temperature Setup
  if (!bmp.begin(0x6A, &customWire)) {
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMP180 Found!");

  // Heartrate Setup
  pinMode(LO_PLUS_PIN, INPUT);   // Setup for leads off detection LO +
  pinMode(LO_MINUS_PIN, INPUT);  // Setup for leads off detection LO -
  Serial.println("Heartrate Sensor Found!");

  // LSM6DSO32 Setup
  if (!dso32.begin_I2C(0x6A, &customWire)) {
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSO32 Found!");

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (dso32.getAccelRange()) {
  case LSM6DSO32_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DSO32_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DSO32_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case LSM6DSO32_ACCEL_RANGE_32_G:
    Serial.println("+-32G");
    break;
  }

  // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (dso32.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSO32
  }

  // dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (dso32.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (dso32.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // GPS setup
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // Initialize GPS with pins
  GPS.begin(9600);  // Initialize GPS at 9600 baud
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Output RMC and GGA data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);                // Request updates on antenna status

  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);  // Ask for firmware version

  Serial.println("GPS Initialized!");

  // Buzzer Setup
  pinMode(BUZZER_PIN, OUTPUT); // configure GPIO8 as an output (can be set to high (buzzer on) or low (buzzer off)

  // LED Setup
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read the sender ID
    int senderID = LoRa.read();
    int numHop = LoRa.read();
    // Read the message ID (4 bytes)
    unsigned long msgID = 0;
    msgID |= (unsigned long)LoRa.read() << 24;
    msgID |= (unsigned long)LoRa.read() << 16;
    msgID |= (unsigned long)LoRa.read() << 8;
    msgID |= (unsigned long)LoRa.read();

    // Read the payload
    String passTemp = LoRa.readString();
    String passAcc = LoRa.readString();
    String passHR = LoRa.readString();
    String passGPS = LoRa.readString();
    int TempAlert = LoRa.read();
    int MotionAlert = LoRa.read();
    int HRAlert = LoRa.read(); 

    // Check for duplicates
    if (isDuplicate(msgID)) {
      Serial.println("Duplicate message. Ignoring.");
      return;
    }

    // Track the new message ID
    trackMessage(msgID);

    // Forward the message to the terminal node
    Serial.println("Forwarding message to terminal node...");
    sendForwardMessage(senderID, numHop, msgID, passTemp, passAcc, passHR, passGPS, TempAlert, MotionAlert, HRAlert);
  }

 // BMP180 Reading
  float temperature = bmp.readTemperature();

  // LSM6DSO32 Reading
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);

  float accelerometer_temperature = temp.temperature;
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float axG = ax*MS2_TO_G;
  float ayG = ay*MS2_TO_G;
  float azG = az*MS2_TO_G;
  float dynamic_acceleration = abs(sqrt(axG*axG + ayG*ayG + azG*azG) - 1.0);

  // Heartrate Reading
  int heartRateValue = 0;
  if ((digitalRead(9) == 1) || (digitalRead(10) == 1)) {
    Serial.println('!');
    heartRateValue = 1;
  } else {
    heartRateValue = analogRead(11);
  }

  // Read data from GPS module
  char c = GPS.read();
  if ((c) && (GPSECHO)) Serial.write(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;  // Fail to parse, so we wait for the next sentence
    }
  }

  Serial.println("Temperature = " + String(temperature));
  Serial.println("Acceleration: x = " + String(ax) + " y = " + String(ay) + " z = " + String(az));
  Serial.println("Dynamic Acceleration = " + String(dynamic_acceleration) + "G");
  Serial.println("Heartrate Measurement = " + String(heartRateValue));
  Serial.println();

  // alertGenerate = checkAlerts(temperature, ax, ay, az, heartRateValue);
  checkAlerts(temperature, ax, ay, az, heartRateValue);


// struct AlertStatus{
//   bool isHRAlert = false; // check if we should generate alert currently for heartrate
//   bool isTempAlert = false; // check if we should generate alert currently for temperature
//   bool isMotionAlert = false; // check if we should generate alert currently for no motion
//   unsigned long hrAlertStart = 0; // when did heartrate start violating threshold
//   unsigned long tempAlertStart = 0; // when did temperature start exceeding threshold
//   float accX = 0.0; // last relevant x acc
//   float accY = 0.0; // last relevant y acc
//   float accZ = 0.0; // last relevant z acc 
//   unsigned long motionAlertStart = 0; // when did motion start becoming stagnant
//   unsigned long lastUpdate = 0; // track last time update was sent to central
//   bool inEmergency = false; // bool of whether current node is in emergency status
// }

  Serial.println("HR Alert = " + String(wearableAlert.isHRAlert));
  Serial.println("Temp Alert = " + String(wearableAlert.isTempAlert));
  Serial.println("Motion Alert = " + String(wearableAlert.isMotionAlert));
  Serial.println("HR Alert Start = " + String(wearableAlert.hrAlertStart));
  Serial.println("Temp Alert Start = " + String(wearableAlert.tempAlertStart));
  Serial.println("Motion Alert Start = " + String(wearableAlert.motionAlertStart));
  Serial.println("In Emergency = " + String(wearableAlert.inEmergency));


  // if (alertGenerate){
  //   playWarningPattern();
  // }
  // if (wearableAlert.inEmergency){
  //   playWarningPattern();
  // }

  unsigned long updateInterval = wearableAlert.inEmergency ? EMERGENCY_UPDATE_FREQ : NORMAL_UPDATE_FREQ;

  // Example: Sending a message from this node periodically
  unsigned long currentTime = millis();
  if (currentTime - wearableAlert.lastUpdate >= updateInterval) { // Every 1 or 0.5 seconds

    Serial.println("Sending Data");
    // If there's a valid GPS fix, send the data
    String gps;
    if (GPS.fix) {
      gps = formatGPSData();  // Get formatted GPS data
    } else{
      gps = "no GPS signal";
    }

    Serial.println(gps);

    sendMessage(temperature, dynamic_acceleration, heartRateValue, gps, wearableAlert.isTempAlert, wearableAlert.isMotionAlert, wearableAlert.isHRAlert);  // ADD GPS
    wearableAlert.lastUpdate = millis();
  }

  delay(100); // Small delay to avoid spamming
}

bool noMovement(float ax, float ay, float az) {
  bool no_movement = (abs(ax - wearableAlert.accX) < MOVEMENT_THRESHOLD ||
                  abs(ay - wearableAlert.accY) < MOVEMENT_THRESHOLD ||
                  abs(az - wearableAlert.accZ) < MOVEMENT_THRESHOLD);
  
  wearableAlert.accX = ax;
  wearableAlert.accY = ay;
  wearableAlert.accZ = az;

  return no_movement;
}

void checkAlerts(float temperature, float ax, float ay, float az, int heartRateValue){
  unsigned long currentTime = millis();
  // bool alertGenerate = false;
  
  // Heart Rate Alert Check
  // check if heart rate is currently above or below thresholds
  if (heartRateValue > HR_MAX || heartRateValue < HR_MIN){
    // if heart rate exceeds threshold and hrAlertStart == 0, set hrAlertStart to currentTime 
    if (wearableAlert.hrAlertStart == 0){
      wearableAlert.hrAlertStart = currentTime;
    } else if (currentTime - wearableAlert.hrAlertStart > HR_ALERT_TIME){
      wearableAlert.isHRAlert = true;
      // alertGenerate = true;
    }
  } else{
    // heart rate doesn't exceed threshold, so set isHRAlert to false and hrAlertStart to 0
    wearableAlert.isHRAlert = false;
    wearableAlert.hrAlertStart = 0;
  }

  // Temperature Alert Check
  // check if temperature exceeds threshold
  if (temperature > TEMP_MAX){
    // if temperature exceeds threshold and isTempAlert == false or tempAlertStart == 0, set tempAlertStart to currentTime
    if (wearableAlert.tempAlertStart == 0){
      wearableAlert.tempAlertStart = currentTime;
    } else if (currentTime - wearableAlert.tempAlertStart > TEMP_ALERT_TIME){
      wearableAlert.isTempAlert = true;
      // alertGenerate = true;
    } 
  } else{
      // temperature doesn't exceed threshold, so set isTempAlert to false and tempAlertStart to 0
      wearableAlert.isTempAlert = false;
      wearableAlert.tempAlertStart = 0;
  }

  // Motion Alert Check
  if (noMovement(ax, ay, az) == true){
    // if there is no movement and motionAlertStart == 0, set motionAlertStart to currentTime
    if(wearableAlert.motionAlertStart == 0){
      wearableAlert.motionAlertStart = currentTime;
    } else if(currentTime - wearableAlert.motionAlertStart > MOTION_ALERT_TIME){
      // if there is no movement for over 60 seconds
      wearableAlert.isMotionAlert = true;
      // alertGenerate = true;
    }
  } else{
    // if there is movement, set isMotionAlert to false and reset motionAlertStart to 0
    wearableAlert.isMotionAlert = false;
    wearableAlert.motionAlertStart = 0;
  }

  // Update Emergency Status
  wearableAlert.inEmergency = wearableAlert.isHRAlert || wearableAlert.isTempAlert || wearableAlert.isMotionAlert;

  // return alertGenerate
}

// Add a message ID to the tracking array
void trackMessage(unsigned long msgID) {
  trackedMsgIDs[trackedIndex] = msgID;
  trackedIndex = (trackedIndex + 1) % MAX_TRACKED_MESSAGES;
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

void playSiren() {
  // Sweep up and down a few times
  for(int i = 0; i < 3; i++) {
    for(int freq = 2000; freq < 4000; freq += 200) {
      playTone(freq, 20);
      delay(5);
    }
    for(int freq = 4000; freq > 2000; freq -= 200) {
      playTone(freq, 20);
      delay(5);
    }
  }
}

void playUrgentBeeps() {
  // Rapid beeps at resonant frequency
  for(int i = 0; i < 8; i++) {
    playTone(4000, 30);
    delay(30);
  }
}

void playWarningPattern() {
  // Combine different warning sounds
  // playSiren();
  // delay(100);
  playUrgentBeeps();
  // delay(100);
  // playSiren();
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

void sendMessage(float temperature, float acceleration, int heartRateValue, String GPS, bool tempAlert, bool motionAlert, bool HRAlert) {
  unsigned long msgID = millis(); // Simple unique ID based on uptime
  int numHop = 1;
  LoRa.beginPacket();
  LoRa.write(NODE_ID);             // Sender ID
  LoRa.write(numHop);             // number of hops
  LoRa.write((byte)(msgID >> 24)); // Message ID (4 bytes)
  LoRa.write((byte)(msgID >> 16));
  LoRa.write((byte)(msgID >> 8));
  LoRa.write((byte)(msgID));
  LoRa.print(temperature);
  LoRa.print(acceleration);
  LoRa.print(heartRateValue);
  LoRa.print(GPS);
  LoRa.write(tempAlert);
  LoRa.write(motionAlert);
  LoRa.write(HRAlert);
  LoRa.endPacket();

  Serial.print("Sent Message ID: ");
  Serial.print(msgID);
  // Serial.print(" Payload: ");
  // Serial.println(payload);

  // Track the sent message to avoid resending
  trackMessage(msgID);
}

// Function to forward a received message
void sendForwardMessage(int originalSender, int numHop, unsigned long msgID, String passTemp, String passAcc, String passHR, String passGPS, int TempAlert, int MotionAlert, int HRAlert) {
  LoRa.beginPacket();
  LoRa.write(NODE_ID);               // Sender ID (this node)
  LoRa.write(numHop+1);               // number of hop the messages have done
  LoRa.write((byte)(msgID >> 24));   // Original Message ID
  LoRa.write((byte)(msgID >> 16));
  LoRa.write((byte)(msgID >> 8));
  LoRa.write((byte)(msgID));
  LoRa.print(passTemp);
  LoRa.print(passAcc);
  LoRa.print(passHR);
  LoRa.print(passGPS);
  LoRa.write(TempAlert);
  LoRa.write(MotionAlert);
  LoRa.write(HRAlert);
  LoRa.endPacket();

  Serial.print("Forwarded Message ID: ");
  Serial.print(msgID);
  Serial.print(" with # of hop =  ");
  Serial.print(numHop);
  // Serial.print(" Payload: ");
  // Serial.println(payload);
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());

  // Track the forwarded message to avoid resending
  trackMessage(msgID);
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

