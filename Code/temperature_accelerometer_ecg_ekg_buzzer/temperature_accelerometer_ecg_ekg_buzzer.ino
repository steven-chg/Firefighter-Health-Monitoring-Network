#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM6DSO32.h>

Adafruit_BMP085 bmp;
Adafruit_LSM6DSO32 dso32;

#define SDA_PIN 37
#define SCL_PIN 38
TwoWire customWire = TwoWire(0); // Create a custom I2C instance on port 0

#define LO_PLUS_PIN 9
#define LO_MINUS_PIN 10
#define BUZZER_PIN 8
#define LED_BUILTIN 18

void setup() {
    // Buzzer Setup
  pinMode(BUZZER_PIN, OUTPUT); // configure GPIO8 as an output (can be set to high (buzzer on) or low (buzzer off)

}
  
// void setup() {
//   Serial.begin(115200);

//   while (!Serial);

//   Serial.println("Sensors Test!");
//   customWire.begin(SDA_PIN, SCL_PIN);

//   // Temperature Setup
//   if (!bmp.begin(0x6A, &customWire)) {
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("BMP180 Found!");

//   // Heartrate Setup
//   pinMode(LO_PLUS_PIN, INPUT);   // Setup for leads off detection LO +
//   pinMode(LO_MINUS_PIN, INPUT);  // Setup for leads off detection LO -
//   Serial.println("Heartrate Sensor Found!");

//   // LSM6DSO32 Setup
//   if (!dso32.begin_I2C(0x6A, &customWire)) {
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("LSM6DSO32 Found!");

//   dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
//   Serial.print("Accelerometer range set to: ");
//   switch (dso32.getAccelRange()) {
//   case LSM6DSO32_ACCEL_RANGE_4_G:
//     Serial.println("+-4G");
//     break;
//   case LSM6DSO32_ACCEL_RANGE_8_G:
//     Serial.println("+-8G");
//     break;
//   case LSM6DSO32_ACCEL_RANGE_16_G:
//     Serial.println("+-16G");
//     break;
//   case LSM6DSO32_ACCEL_RANGE_32_G:
//     Serial.println("+-32G");
//     break;
//   }

//   // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
//   Serial.print("Gyro range set to: ");
//   switch (dso32.getGyroRange()) {
//   case LSM6DS_GYRO_RANGE_125_DPS:
//     Serial.println("125 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_250_DPS:
//     Serial.println("250 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_500_DPS:
//     Serial.println("500 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_1000_DPS:
//     Serial.println("1000 degrees/s");
//     break;
//   case LSM6DS_GYRO_RANGE_2000_DPS:
//     Serial.println("2000 degrees/s");
//     break;
//   case ISM330DHCX_GYRO_RANGE_4000_DPS:
//     break; // unsupported range for the DSO32
//   }

//   // dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
//   Serial.print("Accelerometer data rate set to: ");
//   switch (dso32.getAccelDataRate()) {
//   case LSM6DS_RATE_SHUTDOWN:
//     Serial.println("0 Hz");
//     break;
//   case LSM6DS_RATE_12_5_HZ:
//     Serial.println("12.5 Hz");
//     break;
//   case LSM6DS_RATE_26_HZ:
//     Serial.println("26 Hz");
//     break;
//   case LSM6DS_RATE_52_HZ:
//     Serial.println("52 Hz");
//     break;
//   case LSM6DS_RATE_104_HZ:
//     Serial.println("104 Hz");
//     break;
//   case LSM6DS_RATE_208_HZ:
//     Serial.println("208 Hz");
//     break;
//   case LSM6DS_RATE_416_HZ:
//     Serial.println("416 Hz");
//     break;
//   case LSM6DS_RATE_833_HZ:
//     Serial.println("833 Hz");
//     break;
//   case LSM6DS_RATE_1_66K_HZ:
//     Serial.println("1.66 KHz");
//     break;
//   case LSM6DS_RATE_3_33K_HZ:
//     Serial.println("3.33 KHz");
//     break;
//   case LSM6DS_RATE_6_66K_HZ:
//     Serial.println("6.66 KHz");
//     break;
//   }

//   // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
//   Serial.print("Gyro data rate set to: ");
//   switch (dso32.getGyroDataRate()) {
//   case LSM6DS_RATE_SHUTDOWN:
//     Serial.println("0 Hz");
//     break;
//   case LSM6DS_RATE_12_5_HZ:
//     Serial.println("12.5 Hz");
//     break;
//   case LSM6DS_RATE_26_HZ:
//     Serial.println("26 Hz");
//     break;
//   case LSM6DS_RATE_52_HZ:
//     Serial.println("52 Hz");
//     break;
//   case LSM6DS_RATE_104_HZ:
//     Serial.println("104 Hz");
//     break;
//   case LSM6DS_RATE_208_HZ:
//     Serial.println("208 Hz");
//     break;
//   case LSM6DS_RATE_416_HZ:
//     Serial.println("416 Hz");
//     break;
//   case LSM6DS_RATE_833_HZ:
//     Serial.println("833 Hz");
//     break;
//   case LSM6DS_RATE_1_66K_HZ:
//     Serial.println("1.66 KHz");
//     break;
//   case LSM6DS_RATE_3_33K_HZ:
//     Serial.println("3.33 KHz");
//     break;
//   case LSM6DS_RATE_6_66K_HZ:
//     Serial.println("6.66 KHz");
//     break;
//   }

//   // Buzzer Setup
//   pinMode(BUZZER_PIN, OUTPUT); // configure GPIO8 as an output (can be set to high (buzzer on) or low (buzzer off)

//   // LED Setup
//   pinMode(LED_BUILTIN, OUTPUT);
// }

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

void loop() {
  // // BMP180 Reading
  // Serial.print("BMP Temperature = ");
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");

  // // LSM6DSO32 Reading
  // //  /* Get a new normalized sensor event */
  // sensors_event_t accel;
  // sensors_event_t gyro;
  // sensors_event_t temp;
  // dso32.getEvent(&accel, &gyro, &temp);

  // Serial.print("Accelerometer Temperature = ");
  // Serial.print(temp.temperature);
  // Serial.println(" *C");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("Accel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  // // Heartrate Reading
  // int heartRateValue = 0;
  // if ((digitalRead(9) == 1) || (digitalRead(10) == 1)) {
  //   Serial.println('!');
  //   heartRateValue = 1;
  // } else {
  //   heartRateValue = analogRead(11);
  // }

  // Serial.print("ECG/EKG Data: ");
  // Serial.println(heartRateValue);

  // Serial.println();
  // delay(500);

  Serial.println("Beep!");
  // playBeep(200);
  playWarningPattern();
  delay(500);

  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(500);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(500);                      // wait for a second
}