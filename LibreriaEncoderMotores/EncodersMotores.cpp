#ifndef ENCODERSMOTORES_H
#define ENCODERSMOTORES_H

#include <Arduino.h>
#include <Wire.h>
#include "driver/ledc.h"

// ==================== PINS MOTOR 1 (Maxon - AS5600) ====================
#define EM_M1_RPWM_PIN    25
#define EM_M1_LPWM_PIN    26
#define EM_M1_R_EN_PIN    27
#define EM_M1_L_EN_PIN    14
#define EM_AS5600_SDA_PIN 21
#define EM_AS5600_SCL_PIN 22
#define EM_AS5600_ADDR    0x36
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
#define EM_REQUESTED_PWM_FREQ 1000UL  // 1 kHz
#define EM_APB_FREQ           80000000UL

static const ledc_mode_t  EM_LEDC_MODE  = LEDC_HIGH_SPEED_MODE;
static const ledc_timer_t EM_LEDC_TIMER = LEDC_TIMER_0;
// canales: 0/1 -> motor1 RPWM/LPWM ; 2/3 -> motor2 RPWM/LPWM
static const ledc_channel_t EM_LEDC_CH_M1_R = LEDC_CHANNEL_0;
static const ledc_channel_t EM_LEDC_CH_M1_L = LEDC_CHANNEL_1;
static const ledc_channel_t EM_LEDC_CH_M2_R = LEDC_CHANNEL_2;
static const ledc_channel_t EM_LEDC_CH_M2_L = LEDC_CHANNEL_3;

static uint32_t em_pwmMax = 0;

// ==================== REDUCCIONES ====================
// Motor 1: MAXON con imán AS5600 en el eje del motor
// Reducción motor : salida = 26/7 ≈ 3.714:1
static const double EM_REDUCCION_1 = (26.0/7.0)*60.0;

// Motor 2: Faulhaber (5.42:1) * transmisión externa 2:1
static const double EM_REDUCCION_2 = 5.42 * 2.0;



// (Opcional) pequeña zona muerta en cuentas para evitar drift muy lento
static const int16_t EM_ENC1_DEADBAND = 1;  // ignora diffs -1..+1 (puedes dejarlo en 0 si quieres)

// ==================== ESTADO ENCODER 1 (AS5600, EJE DEL MOTOR) ====================
// Integramos solo con cuentas "crudas" (sin K), como en la versión que sí te funcionaba.

static bool          em_init1         = false;
static uint16_t      em_prevRaw1      = 0;      // última lectura cruda 0..4095
static int32_t       em_rawMulti1     = 0;      // cuentas acumuladas (multi-vuelta motor)
static unsigned long em_prevTime1_us  = 0;      // opcional, para velocidad luego

// ==================== ESTADO ENCODER 2 (incremental) ====================
static const long   EM_PPR_2       = 500;          // pulsos por vuelta por canal
static const int    EM_DECODE_X_2  = 1;            // 1x
static long         EM_CPR_eff_2   = EM_PPR_2 * EM_DECODE_X_2;

static volatile long em_counts2    = 0;
static volatile bool em_dir2_ccw   = true;

static long          em_prevCounts2  = 0;
static unsigned long em_prevTime2_us = 0;
static bool          em_init2        = false;

// ==================== PROTOTIPOS INTERNOS ====================
static void     em_configPWM();
static void     em_applyPWM(int motor, float percentage);
static uint16_t em_readAS5600Raw();
static void IRAM_ATTR em_isr_enc2_a_rising();

// ==================== IMPLEMENTACIÓN INTERNOS ====================

static void em_configPWM() {
  double ratio   = (double)EM_APB_FREQ / (double)EM_REQUESTED_PWM_FREQ;
  double bits_d  = floor(log2(ratio));
  int    maxBits = (int)bits_d;
  if (maxBits > 16) maxBits = 16;
  if (maxBits < 1)  maxBits = 1;
  uint8_t pwmResolution = (uint8_t)maxBits;
  em_pwmMax = ((1UL << pwmResolution) - 1UL);

  ledc_timer_config_t ledc_timer = {
    .speed_mode       = EM_LEDC_MODE,
    .duty_resolution  = (ledc_timer_bit_t)pwmResolution,
    .timer_num        = EM_LEDC_TIMER,
    .freq_hz          = EM_REQUESTED_PWM_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ch;
  ch.speed_mode = EM_LEDC_MODE;
  ch.timer_sel  = EM_LEDC_TIMER;
  ch.intr_type  = LEDC_INTR_DISABLE;
  ch.hpoint     = 0;
  ch.duty       = 0;

  // Motor 1
  ch.gpio_num = EM_M1_RPWM_PIN; ch.channel = EM_LEDC_CH_M1_R;
  ledc_channel_config(&ch);
  ch.gpio_num = EM_M1_LPWM_PIN; ch.channel = EM_LEDC_CH_M1_L;
  ledc_channel_config(&ch);

  // Motor 2
  ch.gpio_num = EM_M2_RPWM_PIN; ch.channel = EM_LEDC_CH_M2_R;
  ledc_channel_config(&ch);
  ch.gpio_num = EM_M2_LPWM_PIN; ch.channel = EM_LEDC_CH_M2_L;
  ledc_channel_config(&ch);

  // Apagar todos
  ledc_set_duty(EM_LEDC_MODE, EM_LEDC_CH_M1_R, 0); ledc_update_duty(EM_LEDC_MODE, EM_LEDC_CH_M1_R);
  ledc_set_duty(EM_LEDC_MODE, EM_LEDC_CH_M1_L, 0); ledc_update_duty(EM_LEDC_MODE, EM_LEDC_CH_M1_L);
  ledc_set_duty(EM_LEDC_MODE, EM_LEDC_CH_M2_R, 0); ledc_update_duty(EM_LEDC_MODE, EM_LEDC_CH_M2_R);
  ledc_set_duty(EM_LEDC_MODE, EM_LEDC_CH_M2_L, 0); ledc_update_duty(EM_LEDC_MODE, EM_LEDC_CH_M2_L);
}

static void em_applyPWM(int motor, float percentage) {
  percentage = constrain(percentage, -100.0f, 100.0f);
  uint32_t val = (uint32_t)((fabs(percentage) / 100.0f) * (double)em_pwmMax + 0.5);
  if (val > em_pwmMax) val = em_pwmMax;

  ledc_channel_t chR, chL;
  if (motor == 1) {
    chR = EM_LEDC_CH_M1_R;
    chL = EM_LEDC_CH_M1_L;
  } else {
    chR = EM_LEDC_CH_M2_R;
    chL = EM_LEDC_CH_M2_L;
  }

  if (percentage >= 0.0f) {
    ledc_set_duty(EM_LEDC_MODE, chR, val);
    ledc_set_duty(EM_LEDC_MODE, chL, 0);
  } else {
    ledc_set_duty(EM_LEDC_MODE, chR, 0);
    ledc_set_duty(EM_LEDC_MODE, chL, val);
  }
  ledc_update_duty(EM_LEDC_MODE, chR);
  ledc_update_duty(EM_LEDC_MODE, chL);
}

static uint16_t em_readAS5600Raw() {
  Wire.beginTransmission(EM_AS5600_ADDR);
  Wire.write(EM_AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(EM_AS5600_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return (uint16_t)(((hi << 8) | lo) & 0x0FFF);  // 12 bits -> 0..4095
  }
  return 0;
}

static void IRAM_ATTR em_isr_enc2_a_rising() {
  int b = gpio_get_level((gpio_num_t)EM_ENC2_B_PIN);
  if (!b) {
    em_counts2++;
    em_dir2_ccw = true;
  } else {
    em_counts2--;
    em_dir2_ccw = false;
  }
}


// ==================== HELPERS INTERNOS PARA ÁNGULOS ====================

// Actualiza el estado del encoder 1 (AS5600) y devuelve ángulo de SALIDA [rad] multi-vuelta
static double em_updateAngle1Rad() {
  if (!em_init1) {
    em_prevTime1_us = micros();
    em_prevRaw1     = em_readAS5600Raw();  // referencia inicial
    em_rawMulti1    = 0;
    em_init1        = true;
  }

  uint16_t raw = em_readAS5600Raw();   // 0..4095

  int16_t diff = (int16_t)raw - (int16_t)em_prevRaw1;
  if (diff >  2048) diff -= 4096;
  if (diff < -2048) diff += 4096;

  // zona muerta opcional para evitar integrar ruido muy pequeño
  if (EM_ENC1_DEADBAND > 0) {
    if (diff > -EM_ENC1_DEADBAND && diff < EM_ENC1_DEADBAND) {
      diff = 0;
    }
  }

  em_rawMulti1 += diff;
  em_prevRaw1   = raw;

  const double K_RAD_PER_COUNT = (2.0 * PI) / 4096.0;
  double angle_motor_rad = (double)em_rawMulti1 * K_RAD_PER_COUNT;
  double angle_out_rad   = angle_motor_rad / EM_REDUCCION_1;

  return angle_out_rad;
}

// Devuelve ángulo de SALIDA del motor 2 [rad] multi-vuelta
static double em_getAngle2Rad() {
  noInterrupts();
  long c = em_counts2;
  interrupts();

  double angle_deg = ((double)c / (double)EM_CPR_eff_2) * (360.0 / EM_REDUCCION_2);
  double angle_rad = angle_deg * (PI / 180.0);
  return angle_rad;
}


// ==================== API PÚBLICA ====================

void EM_begin() {
  em_configPWM();

  // EN pins BTS7960
  pinMode(EM_M1_R_EN_PIN, OUTPUT);
  pinMode(EM_M1_L_EN_PIN, OUTPUT);
  digitalWrite(EM_M1_R_EN_PIN, HIGH);
  digitalWrite(EM_M1_L_EN_PIN, HIGH);

  pinMode(EM_M2_R_EN_PIN, OUTPUT);
  pinMode(EM_M2_L_EN_PIN, OUTPUT);
  digitalWrite(EM_M2_R_EN_PIN, HIGH);
  digitalWrite(EM_M2_L_EN_PIN, HIGH);

  // I2C AS5600
  Wire.begin(EM_AS5600_SDA_PIN, EM_AS5600_SCL_PIN);
  Wire.setClock(400000);

  // Encoder incremental motor 2
  pinMode(EM_ENC2_A_PIN, INPUT_PULLUP);
  pinMode(EM_ENC2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EM_ENC2_A_PIN), em_isr_enc2_a_rising, RISING);

  // Estado encoder 1
  em_init1        = false;
  em_prevRaw1     = 0;
  em_rawMulti1    = 0;
  em_prevTime1_us = micros();

  // Estado encoder 2
  em_prevTime2_us = micros();
  em_prevCounts2  = 0;
  em_init2        = false;

  Serial.println(F("[EM] EM_begin() listo."));
}

void EM_controlMotor1(float percentage) {
  em_applyPWM(1, percentage);
}

void EM_controlMotor2(float percentage) {
  em_applyPWM(2, percentage);
}

// ==================== ENCODER 1 (AS5600) ====================
// Lee el ángulo de salida (después de la reducción EM_REDUCCION_1)
// en radianes, multi-vuelta, y aplica una escala distinta a cada lado
// sin tocar la integración interna (no debe derivar por cambiar K).
void EM_printEncoder1() {
  double angle_out_rad = em_updateAngle1Rad();
  Serial.println(angle_out_rad, 6);
}


// ==================== ENCODER 2 (incremental) ====================

void EM_printEncoder2() {
  if (!em_init2) {
    em_prevTime2_us = micros();
    em_prevCounts2  = 0;
    em_init2        = true;
  }

  double angle_rad = em_getAngle2Rad();
  Serial.println(angle_rad, 6);
}



// ==================== TEST DE PICOS DE PWM - MOTOR 1 ====================
//
// Envía una serie de pulsos de PWM (duty fijo, ancho creciente),
// alternando la dirección (+/-), y mide:
//  - tiempo de inicio
//  - tiempo hasta que el ángulo se estabiliza
//  - ángulo inicial y final
//
// IMPORTANTE: esta función es bloqueante; llama una sola vez desde loop().

void EM_testPicosMotor1() {
  const float duty        = 30.0f;   // % de PWM que se aplicará en cada pulso
  const uint32_t base_ms  = 30;      // ancho del primer pulso [ms]
  const uint32_t step_ms  = 20;      // incremento de ancho por pulso [ms]
  const int nPulses       = 10;      // número total de pulsos
  const double eps_rad    = 0.002;   // ~0.1°: umbral para considerar cambio de ángulo
  const uint32_t quiet_ms = 80;      // ventana sin cambios para decir "se estabilizó"
  const uint32_t max_ms   = 4000;    // timeout de seguridad por pulso

  Serial.println(F("#TEST PULSOS M1"));
  Serial.println(F("#pulso,dir,width_ms,t_start_ms,t_settle_ms,angle_ini_rad,angle_fin_rad"));

  for (int i = 0; i < nPulses; ++i) {
    int dir = (i % 2 == 0) ? +1 : -1;   // alterna +, -, +, -, ...
    uint32_t width_ms = base_ms + (uint32_t)i * step_ms;

    // Lee ángulo inicial y tiempo de inicio
    double angle_ini = em_updateAngle1Rad();
    unsigned long t0 = millis();

    // Imprime info del pulso
    Serial.print(i);
    Serial.print(",");
    Serial.print(dir);
    Serial.print(",");
    Serial.print(width_ms);
    Serial.print(",");
    Serial.print(t0);
    Serial.print(",");

    // Aplica PWM en la dirección dada
    EM_controlMotor1(dir * duty);
    delay(width_ms);   // mantener el pulso
    EM_controlMotor1(0.0f);  // cortar PWM

    // Ahora esperar a que el motor se detenga (ángulo deje de cambiar)
    unsigned long t_lastChange = millis();
    double lastAngle = em_updateAngle1Rad();

    while (true) {
      delay(5);
      double currentAngle = em_updateAngle1Rad();

      if (fabs(currentAngle - lastAngle) > eps_rad) {
        lastAngle = currentAngle;
        t_lastChange = millis();
      }

      unsigned long now = millis();
      if (now - t_lastChange > quiet_ms) {
        // llevamos quiet_ms sin cambios significativos -> consideramos posición final
        break;
      }

      if (now - t0 > max_ms) {
        // timeout de seguridad
        break;
      }
    }

    unsigned long t_settle = millis();
    double angle_fin = em_updateAngle1Rad();

    // Completar la línea con t_settle y ángulos
    Serial.print(t_settle);
    Serial.print(",");
    Serial.print(angle_ini, 6);
    Serial.print(",");
    Serial.println(angle_fin, 6);
  }

  Serial.println(F("#FIN TEST M1"));
}


// ==================== TEST DE PICOS DE PWM - MOTOR 2 ====================
//
// Misma idea que M1, pero usando el encoder incremental (em_counts2).
// Ángulos de salida teniendo en cuenta EM_REDUCCION_2.

void EM_testPicosMotor2() {
  const float duty        = 30.0f;   // % de PWM
  const uint32_t base_ms  = 30;      // ancho inicial [ms]
  const uint32_t step_ms  = 20;      // incremento por pulso [ms]
  const int nPulses       = 10;      // cantidad de pulsos
  const double eps_rad    = 0.002;   // umbral de cambio de ángulo
  const uint32_t quiet_ms = 80;      // ventana sin cambios
  const uint32_t max_ms   = 4000;    // timeout

  Serial.println(F("#TEST PULSOS M2"));
  Serial.println(F("#pulso,dir,width_ms,t_start_ms,t_settle_ms,angle_ini_rad,angle_fin_rad"));

  for (int i = 0; i < nPulses; ++i) {
    int dir = (i % 2 == 0) ? +1 : -1;
    uint32_t width_ms = base_ms + (uint32_t)i * step_ms;

    // Asegurar init2 si aún no se había usado
    if (!em_init2) {
      em_prevTime2_us = micros();
      em_prevCounts2  = 0;
      em_init2        = true;
    }

    double angle_ini = em_getAngle2Rad();
    unsigned long t0 = millis();

    Serial.print(i);
    Serial.print(",");
    Serial.print(dir);
    Serial.print(",");
    Serial.print(width_ms);
    Serial.print(",");
    Serial.print(t0);
    Serial.print(",");

    EM_controlMotor2(dir * duty);
    delay(width_ms);
    EM_controlMotor2(0.0f);

    unsigned long t_lastChange = millis();
    double lastAngle = em_getAngle2Rad();

    while (true) {
      delay(5);
      double currentAngle = em_getAngle2Rad();

      if (fabs(currentAngle - lastAngle) > eps_rad) {
        lastAngle = currentAngle;
        t_lastChange = millis();
      }

      unsigned long now = millis();
      if (now - t_lastChange > quiet_ms) {
        break;
      }
      if (now - t0 > max_ms) {
        break;
      }
    }

    unsigned long t_settle = millis();
    double angle_fin = em_getAngle2Rad();

    Serial.print(t_settle);
    Serial.print(",");
    Serial.print(angle_ini, 6);
    Serial.print(",");
    Serial.println(angle_fin, 6);
  }

  Serial.println(F("#FIN TEST M2"));
}


#endif // ENCODERSMOTORES_H
