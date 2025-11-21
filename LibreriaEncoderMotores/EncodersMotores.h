#ifndef ENCODERS_MOTORES_H
#define ENCODERS_MOTORES_H

#include <Arduino.h>
#include <Wire.h>
#include "driver/ledc.h"

// ==================== PINS MOTOR 1 (Maxon - AS5600) ====================
#define EM_M1_RPWM_PIN      25
#define EM_M1_LPWM_PIN      26
#define EM_M1_R_EN_PIN      27
#define EM_M1_L_EN_PIN      14
#define EM_AS5600_SDA_PIN   21
#define EM_AS5600_SCL_PIN   22
#define EM_AS5600_ADDR      0x36
#define EM_AS5600_RAW_ANGLE_H 0x0C
#define EM_AS5600_RAW_ANGLE_L 0x0D

// ==================== PINS MOTOR 2 (Faulhaber - incremental encoder) ====================
#define EM_M2_RPWM_PIN    16
#define EM_M2_LPWM_PIN    17
#define EM_M2_R_EN_PIN    2
#define EM_M2_L_EN_PIN    4
#define EM_ENC2_A_PIN     32
#define EM_ENC2_B_PIN     33

// ==================== PWM / LEDC ====================
#define EM_REQUESTED_PWM_FREQ 1000UL     // 1 kHz
#define EM_APB_FREQ           80000000UL // 80 MHz

// ==================== PARÁMETROS ENCODERS ====================
// Encoder 1 (AS5600 + reductora)
#define EM_REDUCCION_1    ((26.0/7.0)*60.0)      // cambia según tu reductora real
#define EM_ENC1_DEADBAND  0        // cuentas crudas para zona muerta

// Encoder 2 (incremental)
#define EM_PPR_2        500        // pulsos por vuelta por canal
#define EM_DECODE_X_2   1          // 1x
#define EM_REDUCCION_2  ((141.0/26.0)*2.0)        // cambia según tu reductora real

// ==================== API PÚBLICA ====================
void   EM_begin();

// Control de motores: porcentaje [-100, 100]
void   EM_controlMotor1(float percentage);
void   EM_controlMotor2(float percentage);

// Lecturas de ángulo (salida) en grados, multi-vuelta
double EM_getAngle1Deg();
double EM_getAngle2Deg();

// Helpers que imprimen por Serial el ángulo en grados
double   EM_printEncoder1();
double   EM_printEncoder2();

// Tests de picos de PWM (como ya los tenías)
void   EM_testPicosMotor1();
void   EM_testPicosMotor2();

#endif // ENCODERS_MOTORES_H
