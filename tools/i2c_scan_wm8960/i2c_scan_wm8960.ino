/*
  I2C scanner for the ReSpeaker 2-Mic HAT (WM8960 codec at 0x1A) on an ESP32-S3 dev board.

  Wiring (Pi header on the HAT -> dev board):
    pin 1  3.3 V  -> 3V3
    pin 2  5 V    -> 5V
    pin 6  GND    -> GND
    pin 3  SDA    -> GPIO8   (+ 4.7k pull-up to 3.3 V)
    pin 5  SCL    -> GPIO9   (+ 4.7k pull-up to 3.3 V)
  Nothing else needs connecting for this test.

  Serial Monitor: 115200 baud, "New Line".
*/
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(SDA_PIN, INPUT);   // read the idle levels before I2C takes the pins
  pinMode(SCL_PIN, INPUT);
  delay(10);
  Serial.printf("\nIdle levels: SDA=%s  SCL=%s  (both should be HIGH)\n",
                digitalRead(SDA_PIN) ? "HIGH" : "LOW", digitalRead(SCL_PIN) ? "HIGH" : "LOW");
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
}

void loop() {
  Serial.println("Scanning I2C...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found device at 0x%02X%s\n", addr, addr == 0x1A ? "  <-- WM8960 codec: the HAT is alive" : "");
      found++;
    }
  }
  if (!found) Serial.println("  nothing answered");
  delay(3000);
}
