// #include <ArduinoBLE.h>

// // BLE service and characteristic definitions
// BLEDevice peripheral;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   if (!BLE.begin()) {
//     Serial.println("starting BLE failed!");
//     while (1);
//   }

//   Serial.println("BLE Central - Heart Rate Monitor");
//   BLE.scanForUuid("180D");
// }

// void loop() {
//   peripheral = BLE.available();
  
//   if (peripheral) {
//     if (peripheral.localName() == "HRM-Dual:084939") { // Update this to match your device name
//       BLE.stopScan();
      
//       if (peripheral.connect()) {
//         if (peripheral.discoverAttributes()) {
//           BLEService service = peripheral.service("180d");
//           BLECharacteristic characteristic = service.characteristic("2a37");
//           characteristic.subscribe();
          
//           while (peripheral.connected()) {
//             if (characteristic.valueUpdated()) {
//               uint8_t value[6];
//               characteristic.readValue(value, 6);
//               Serial.println(value[1]); // Only print heart rate value
//             }
//             delay(100);
//           }
//         }
//         peripheral.disconnect();
//       }
//     }
//     BLE.scanForUuid("180D");
//   }
// }



// #include <ArduinoBLE.h>

// BLEDevice peripheral;
// int lastHeartRate = 0;  // Store the last valid heart rate reading

// void setup() {
//   Serial.begin(115200);
//   if (!BLE.begin()) {
//     Serial.println("Starting BLE failed!");
//     return;
//   }

//   Serial.println("BLE Heart Rate Monitor initialized");
//   BLE.scanForUuid("180D");
// }

// int getHeartRate() {
//   peripheral = BLE.available();
  
//   if (peripheral) {
//     if (peripheral.localName() == "HRM-Dual:084939") {  // Update this to match your device name
//       BLE.stopScan();
      
//       if (peripheral.connect()) {
//         if (peripheral.discoverAttributes()) {
//           BLEService service = peripheral.service("180d");
//           BLECharacteristic characteristic = service.characteristic("2a37");
//           characteristic.subscribe();
          
//           // no outer while(peripheral.connected()){}
//           if (characteristic.valueUpdated()) {
//             uint8_t value[6];
//             characteristic.readValue(value, 6);
//             lastHeartRate = value[1];
//             return lastHeartRate;
//           }
//         }
//         peripheral.disconnect();
//       }
//       BLE.scanForUuid("180D");
//     }
//   }
  
//   return lastHeartRate;  // Return the last known heart rate if no new reading
// }

// // Example of how to use in your mesh network loop
// void loop() {
//     // Get heart rate without blocking
//     int heartRate = getHeartRate();
    
//     Serial.print("Heart Rate: ");
//     Serial.println(heartRate);
//     if (heartRate > 0) {
//         Serial.print("Heart Rate: ");
//         Serial.println(heartRate);
//     }
    
//     delay(100);  // Small delay to prevent overwhelming the BLE connection
// }










#include <ArduinoBLE.h>

BLEDevice peripheral;
BLECharacteristic heartRateCharacteristic;
bool isConnected = false;
int lastHeartRate = 0;

void setup() {
  Serial.begin(115200);
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    return;
  }
  Serial.println("BLE Heart Rate Monitor initialized");
  BLE.scanForUuid("180D");
}

bool connectToHRM() {
  if (isConnected) return true;
  
  peripheral = BLE.available();
  
  if (peripheral) {
    if (peripheral.localName() == "HRM-Dual:084939") {
      BLE.stopScan();
      Serial.println("Found HRM device");
      
      if (peripheral.connect()) {
        Serial.println("Connected to HRM");
        
        if (peripheral.discoverAttributes()) {
          BLEService service = peripheral.service("180d");
          heartRateCharacteristic = service.characteristic("2a37");
          
          if (heartRateCharacteristic.subscribe()) {
            isConnected = true;
            Serial.println("Subscribed to heart rate characteristic");
            return true;
          }
        }
      }
      peripheral.disconnect();
    }
  }
  
  isConnected = false;
  BLE.scanForUuid("180D");
  return false;
}

int getHeartRate() {
  if (!isConnected) {
    if (!connectToHRM()) {
      return lastHeartRate;
    }
  }
  
  if (!peripheral.connected()) {
    Serial.println("Lost connection");
    isConnected = false;
    return lastHeartRate;
  }
  
  if (heartRateCharacteristic.valueUpdated()) {
    uint8_t value[6];
    heartRateCharacteristic.readValue(value, 6);
    lastHeartRate = value[1];
    Serial.print("New heart rate: ");
    Serial.println(lastHeartRate);
  }
  
  return lastHeartRate;
}

// Example usage in loop
void loop() {
    int heartRate = getHeartRate();
    // Your other mesh network code here
    delay(100);
}