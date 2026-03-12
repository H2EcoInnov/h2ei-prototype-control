#include <Arduino.h>

const int fcVoltagePin = 27;

// Nombre d'échantillons utilisé pour lisser les mesures analogiques
const int ANALOG_SAMPLES = 20;

const float ADC_REF_VOLTAGE = 3.3f;
const int   ADC_MAX_VALUE   = 4095;   // résolution 12 bit

float readAverageAdcVoltage(int pin, int samples = ANALOG_SAMPLES) {
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }

  float averageCounts = static_cast<float>(sum) / samples;
  return averageCounts * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
}


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  analogReadResolution(12);
  analogReadAveraging(16);
}

void loop() {
  int raw = analogRead(fcVoltagePin);
  float vAdc = raw * 3.3f / 4095.0f;

  float funcV = readAverageAdcVoltage(27);

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" | vAdc=");
  Serial.println(vAdc, 4);
  Serial.print(" | funcv=");
  Serial.println(funcV, 4);

  delay(500);
}