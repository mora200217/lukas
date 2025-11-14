#include <Arduino.h>
#include "EncodersMotores.h"

void setup() {
  Serial.begin(2000000);
  delay(200);
  Serial.println("\n=== Prueba librería EncodersMotores (simple) ===");

  EM_begin();  // inicializa PWM, I2C, encoder, etc.
}

void loop() {
  // 1) Mandar comandos a los motores (en %)
  EM_controlMotor1(30.0f);   // motor 1 al 30% adelante
  EM_controlMotor2(-20.0f);  // motor 2 al 20% atrás

  // 2) Imprimir lecturas por Serial (CSV)
  EM_printEncoder1();  // imprime: M1,angle_deg,vel_deg_s,raw
  EM_printEncoder2();  // imprime: M2,angle_deg,vel_deg_s,counts

  delay(50);  // ajusta según qué tan rápido quieras imprimir
}
