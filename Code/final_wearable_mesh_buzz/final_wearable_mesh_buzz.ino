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
#define NODE_ID 2          // Unique ID for this node

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

// Buzzer timer setup
hw_timer_t* buzzerTimer = NULL;
volatile bool buzzerActive = false;
volatile int currentFreq = 0;
volatile unsigned long buzzerEndTime = 0;

int counter = 0;

// Array to store recent message IDs
unsigned long trackedMsgIDs[MAX_TRACKED_MESSAGES];
int trackedIndex = 0;

// Alert thesholds
const int HR_MAX = 150;
const int HR_MIN = -2;              // CHANGE BACK TO 40
const float TEMP_MAX = 27.0;        // CHANGE BACK TO 40
const float MOVEMENT_THRESHOLD = 0.5;
const float MS2_TO_G = 1.0 / 9.81;

// Alert timing thresholds
const unsigned long HR_ALERT_TIME = 30000;  // 30 seconds
const unsigned long TEMP_ALERT_TIME = 60000; // 3 minutes   // CHANGE BACK TO 3 minutes 180000
const unsigned long MOTION_ALERT_TIME = 60000;  // 60 seconds

// GPS update frequency
const unsigned long NORMAL_UPDATE_FREQ = 1000; // 1 second
const unsigned long EMERGENCY_UPDATE_FREQ = 500; // half a second

struct AlertStatus {
  bool isHRAlert = false;
  bool isTempAlert = false;
  bool isMotionAlert = false;
  unsigned long hrAlertStart = 0;
  unsigned long tempAlertStart = 0;
  float accX = 0.0;
  float accY = 0.0;
  float accZ = 0.0;
  unsigned long motionAlertStart = 0;
  unsigned long lastUpdate = 0;
  bool inEmergency = false;
};

// Structure to hold message details
struct LoRaMessage {
  int senderID;
  unsigned long msgID;
  String payload;
};

AlertStatus wearableAlert;

// Timer interrupt handler for buzzer
void IRAM_ATTR onBuzzerTimer() {
    static bool buzzerState = false;
    if (millis() >= buzzerEndTime) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerActive = false;
        timerAlarmDisable(buzzerTimer);
        return;
    }
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
}

// Non-blocking version of playTone
void playToneNB(int frequency, int duration) {
    if (buzzerActive) return;
    
    buzzerActive = true;
    currentFreq = frequency;
    buzzerEndTime = millis() + duration;
    
    unsigned long period = 1000000 / frequency;
    timerAlarmWrite(buzzerTimer, period / 2, true);
    timerAlarmEnable(buzzerTimer);
}

// Non-blocking version of playSiren
void playSirenNB() {
    static int sirenStep = 0;
    static int currentFreq = 2000;
    static bool ascending = true;
    
    if (!buzzerActive) {
        if (ascending) {
            currentFreq += 200;
            if (currentFreq >= 4000) ascending = false;
        } else {
            currentFreq -= 200;
            if (currentFreq <= 2000) {
                ascending = true;
                sirenStep++;
            }
        }
        
        if (sirenStep < 3) {
            playToneNB(currentFreq, 20);
        } else {
            sirenStep = 0;
        }
    }
}

// Non-blocking version of playUrgentBeeps
void playUrgentBeepsNB() {
    static int beepCount = 0;
    static unsigned long lastBeepTime = 0;
    const int BEEP_INTERVAL = 60; // 30ms on + 30ms off
    
    if (!buzzerActive && (millis() - lastBeepTime >= BEEP_INTERVAL)) {
        if (beepCount < 8) {
            playToneNB(4000, 30);
            beepCount++;
            lastBeepTime = millis();
        } else {
            beepCount = 0;
        }
    }
}

// Non-blocking version of playWarningPattern
void playWarningPatternNB() {
    static unsigned long lastPatternTime = 0;
    const unsigned long PATTERN_INTERVAL = 1000;
    
    if (millis() - lastPatternTime >= PATTERN_INTERVAL) {
        playUrgentBeepsNB();
        lastPatternTime = millis();
    }
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    while (!Serial);

    // Initialize LoRa
    SPI.begin(36, 35, 48, 47);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    if (!LoRa.begin(915E6)) {
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
        while (1) delay(10);
    }
    Serial.println("BMP180 Found!");

    // Heartrate Setup
    pinMode(LO_PLUS_PIN, INPUT);
    pinMode(LO_MINUS_PIN, INPUT);
    Serial.println("Heartrate Sensor Found!");

    // LSM6DSO32 Setup
    if (!dso32.begin_I2C(0x6A, &customWire)) {
        while (1) delay(10);
    }
    Serial.println("LSM6DSO32 Found!");
    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);

    // GPS setup
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);
    gpsSerial.println(PMTK_Q_RELEASE);

    // Buzzer Setup
    pinMode(BUZZER_PIN, OUTPUT);
    buzzerTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(buzzerTimer, &onBuzzerTimer, true);

    // LED Setup
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // Check for incoming LoRa packets
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        handleLoRaPacket();
    }

    // Read sensors
    float temperature = bmp.readTemperature();
    sensors_event_t accel, gyro, temp;
    dso32.getEvent(&accel, &gyro, &temp);

    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;
    float dynamic_acceleration = abs(sqrt(pow(ax*MS2_TO_G, 2) + 
                                       pow(ay*MS2_TO_G, 2) + 
                                       pow(az*MS2_TO_G, 2)) - 1.0);

    // Heartrate Reading
    int heartRateValue = readHeartRate();

    // GPS Reading
    handleGPS();

    // Print sensor data
    printSensorData(temperature, ax, ay, az, dynamic_acceleration, heartRateValue);

    // Check alerts
    checkAlerts(temperature, ax, ay, az, heartRateValue);

    // Handle emergency state
    if (wearableAlert.inEmergency) {
        playWarningPatternNB();
    }

    // Send periodic updates
    handlePeriodicUpdates(temperature, dynamic_acceleration, heartRateValue);

    delay(100);
}

void handleLoRaPacket() {
    int senderID = LoRa.read();
    int numHop = LoRa.read();
    
    unsigned long msgID = 0;
    msgID |= (unsigned long)LoRa.read() << 24;
    msgID |= (unsigned long)LoRa.read() << 16;
    msgID |= (unsigned long)LoRa.read() << 8;
    msgID |= (unsigned long)LoRa.read();

    float passTemp = (float)LoRa.read();
    float passAcc = (float)LoRa.read();
    int passHR = (int)LoRa.read();
    int TempAlert = (int)LoRa.read();
    int MotionAlert = (int)LoRa.read();
    int HRAlert = (int)LoRa.read();
    String passGPS = LoRa.readString();

    if (isDuplicate(msgID)) {
        Serial.println("Duplicate message. Ignoring.");
        return;
    }

    trackMessage(msgID);
    sendForwardMessage(senderID, numHop, msgID, passTemp, passAcc, passHR, 
                      passGPS, TempAlert, MotionAlert, HRAlert);
}

int readHeartRate() {
    if ((digitalRead(9) == 1) || (digitalRead(10) == 1)) {
        Serial.println("No Heart Rate Reading");
        return -1;
    }
    return analogRead(11);
}

void handleGPS() {
    char c = GPS.read();
    if ((c) && (GPSECHO)) Serial.write(c);
    if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA());
    }
}

void printSensorData(float temperature, float ax, float ay, float az, 
                    float dynamic_acceleration, int heartRateValue) {
    Serial.println("Temperature = " + String(temperature));
    Serial.println("Acceleration: x = " + String(ax) + " y = " + String(ay) + 
                  " z = " + String(az));
    Serial.println("Dynamic Acceleration = " + String(dynamic_acceleration) + "G");
    Serial.println("Heartrate Measurement = " + String(heartRateValue));
    Serial.println();
}

void handlePeriodicUpdates(float temperature, float acceleration, int heartRateValue) {
    unsigned long updateInterval = wearableAlert.inEmergency ? 
                                 EMERGENCY_UPDATE_FREQ : NORMAL_UPDATE_FREQ;

    unsigned long currentTime = millis();
    if (currentTime - wearableAlert.lastUpdate >= updateInterval) {
        String gps = GPS.fix ? formatGPSData() : "no GPS signal";
        sendMessage(temperature, acceleration, heartRateValue, gps, 
                   wearableAlert.isTempAlert, wearableAlert.isMotionAlert, 
                   wearableAlert.isHRAlert);
        wearableAlert.lastUpdate = currentTime;
    }
}

// [Previous helper functions remain the same: noMovement, checkAlerts, 
// trackMessage, isDuplicate, sendMessage, sendForwardMessage, formatGPSData]
// Adding them here for completeness but truncated for space

bool noMovement(float ax, float ay, float az) {
    bool no_movement = (abs(ax - wearableAlert.accX) < MOVEMENT_THRESHOLD ||
                       abs(ay - wearableAlert.accY) < MOVEMENT_THRESHOLD ||
                       abs(az - wearableAlert.accZ) < MOVEMENT_THRESHOLD);
    
    wearableAlert.accX = ax;
    wearableAlert.accY = ay;
    wearableAlert.accZ = az;

    return no_movement;
}

void checkAlerts(float temperature, float ax, float ay, float az, int heartRateValue) {
    unsigned long currentTime = millis();
    
    // Heart Rate Alert Check
    if (heartRateValue > HR_MAX || heartRateValue < HR_MIN) {
        if (wearableAlert.hrAlertStart == 0) {
            wearableAlert.hrAlertStart = currentTime;
        } else if (currentTime - wearableAlert.hrAlertStart > HR_ALERT_TIME) {
            wearableAlert.isHRAlert = true;
        }
    } else {
        wearableAlert.isHRAlert = false;
        wearableAlert.hrAlertStart = 0;
    }

    // Temperature Alert Check
    if (temperature > TEMP_MAX) {
        if (wearableAlert.tempAlertStart == 0) {
            wearableAlert.tempAlertStart = currentTime;
        } else if (currentTime - wearableAlert.tempAlertStart > TEMP_ALERT_TIME) {
            wearableAlert.isTempAlert = true;
        }
    } else {
        wearableAlert.isTempAlert = false;
        wearableAlert.tempAlertStart = 0;
    }

    // Motion Alert Check
    if (noMovement(ax, ay, az)) {
        if (wearableAlert.motionAlertStart == 0) {
            wearableAlert.motionAlertStart = currentTime;
        } else if (currentTime - wearableAlert.motionAlertStart > MOTION_ALERT_TIME) {
            wearableAlert.isMotionAlert = true;
        }
    } else {
        wearableAlert.isMotionAlert = false;
        wearableAlert.motionAlertStart;
    }