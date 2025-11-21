#include <Arduino.h>
#include "EncodersMotores.h"

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  EM_begin();

  // Motor 2 al 30% en un sentido:
  //EM_controlMotor2(30.0f);
}

void loop() {
  uint16_t e1 = (uint16_t) EM_printEncoder1();  // función que devuelva el valor en entero
  uint16_t e2 = (uint16_t) EM_printEncoder2();  // lo mismo para el 2

  enviarFrame(e1, e2);
  delay(50);
}

void enviarFrame(uint16_t enc1, uint16_t enc2) {
  // Byte de inicio (header)
  Serial.write(0xFF);

  // Encoder 1 (MSB primero, luego LSB)
  Serial.write((uint8_t)(enc1 >> 8));     // parte alta
  Serial.write((uint8_t)(enc1 & 0xFF));   // parte baja

  // Encoder 2
  Serial.write((uint8_t)(enc2 >> 8));     
  Serial.write((uint8_t)(enc2 & 0xFF));
}
