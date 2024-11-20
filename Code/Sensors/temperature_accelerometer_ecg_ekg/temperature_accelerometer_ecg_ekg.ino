#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM6DSO32.h>
#define seaLevelPressure_hPa 1013.25

Adafruit_BMP085 bmp;
Adafruit_LSM6DSO32 dso32;

#define SDA_PIN 37
#define SCL_PIN 38
TwoWire customWire = TwoWire(0); // Create a custom I2C instance on port 0

#define LO_PLUS_PIN 9
#define LO_MINUS_PIN 10
  
void setup() {
  Serial.begin(9600);

  while (!Serial)
    delay(10); 

  Serial.println("Sensors Test!");

  customWire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin(0x6A, &customWire)) {
    while (1) {
      delay(10);
    }
  }

  Serial.println("BMP180 Found!");

  // Heartrate Setup
  pinMode(LO_PLUS_PIN, INPUT);   // Setup for leads off detection LO +
  pinMode(LO_MINUS_PIN, INPUT);  // Setup for leads off detection LO -

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
}

void loop() {
  // BMP180 Reading
  Serial.print("BMP Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  
  // Serial.print("Pressure = ");
  // Serial.print(bmp.readPressure());
  // Serial.println(" Pa");

  // Serial.print("Altitude = ");
  // Serial.print(bmp.readAltitude());
  // Serial.println(" meters");

  // Serial.print("Pressure at sealevel (calculated) = ");
  // Serial.print(bmp.readSealevelPressure());
  // Serial.println(" Pa");

  // Serial.print("Real altitude = ");
  // Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
  // Serial.println(" meters");

  // LSM6DSO32 Reading
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);

  Serial.print("Accelerometer Temperature = ");
  Serial.print(temp.temperature);
  Serial.println(" *C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  // /* Display the results (rotation is measured in rad/s) */
  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();

  // // serial plotter friendly format
  // Serial.print(temp.temperature);
  // Serial.print(",");

  // Serial.print(accel.acceleration.x);
  // Serial.print(","); Serial.print(accel.acceleration.y);
  // Serial.print(","); Serial.print(accel.acceleration.z);
  // Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  // delayMicroseconds(10000);

  // Heartrate Reading
  int heartRateValue = 0;
  if ((digitalRead(9) == 1) || (digitalRead(10) == 1)) {
    Serial.println('!');
    heartRateValue = 1;
  } else {
    heartRateValue = analogRead(11);
  }

  Serial.print("ECG/EKG Data: ");
  Serial.println(heartRateValue);

  Serial.println();
  delay(500);
}