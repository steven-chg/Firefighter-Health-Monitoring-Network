#include <SPI.h>

void setup() {
  // put your setup code here, to run once:
  SPI.begin(36, 35, 48, 47);
  Serial.begin(115200);
  Serial.print("MOSI:pp ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);  
}

void loop() {
  // put your main code here, to run repeatedly:
}