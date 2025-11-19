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

// ==================== ESTADO ENCODER 1 (AS5600) ====================
static double        em_prevAngle1_deg  = 0.0;
static unsigned long em_prevTime1_us    = 0;
static bool          em_init1           = false;
static const double EM_REDUCCION_1 = 26.0/7.0;  // Reduccion MAXON 3.714:1 

// ==================== ESTADO ENCODER 2 (incremental) ====================
// Encoder parameters (ajusta según tu encoder / reducción)
static const long   EM_PPR_2       = 500;      // pulsos por vuelta por canal
static const int    EM_DECODE_X_2  = 1;        // 1x
static long         EM_CPR_eff_2   = EM_PPR_2 * EM_DECODE_X_2;
static const double EM_REDUCCION_2 = 5.42*2;      // Faulhaber 5.42 en la caja y 2 de la transmision

static volatile long em_counts2    = 0;
static volatile bool em_dir2_ccw   = true;

static long         em_prevCounts2 = 0;
static unsigned long em_prevTime2_us = 0;
static bool          em_init2        = false;

// ==================== PROTOTIPOS INTERNOS ====================
static void     em_configPWM();
static void     em_applyPWM(int motor, float percentage);
static uint16_t em_readAS5600Raw();
static void IRAM_ATTR em_isr_enc2_a_rising();

// ==================== IMPLEMENTACIÓN INTERNOS ====================

static void em_configPWM() {
  // Calcula resolución de PWM a partir de APB_FREQ y frecuencia deseada
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

  // Asegurar que todos arrancan apagados
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
    // Adelante
    ledc_set_duty(EM_LEDC_MODE, chR, val);
    ledc_set_duty(EM_LEDC_MODE, chL, 0);
  } else {
    // Atrás
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
    return (uint16_t)(((hi << 8) | lo) & 0x0FFF);
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

// ==================== API PÚBLICA ====================

// Llamar una vez en setup (después de Serial.begin)
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

  // Inicializar estados
  em_prevTime1_us = micros();
  em_prevAngle1_deg = 0.0;
  em_init1 = true;

  em_prevTime2_us = micros();
  em_prevCounts2 = 0;
  em_init2 = true;

  // Mensaje opcional
  Serial.println(F("[EM] EM_begin() listo."));
}

// Control motor 1 (porcentaje -100 a 100)
 void EM_controlMotor1(float percentage) {
  em_applyPWM(1, percentage);
}

// Control motor 2 (porcentaje -100 a 100)
 void EM_controlMotor2(float percentage) {
  em_applyPWM(2, percentage);
}

// Imprime datos del encoder 1 (AS5600) en CSV:
// M1,angle_deg,vel_deg_s,raw
double EM_printEncoder1() {
    if (!em_init1) {
        em_prevTime1_us   = micros();
        em_prevAngle1_deg = 0.0;
        em_init1          = true;
    }

    unsigned long t_us = micros();
    uint16_t raw = em_readAS5600Raw();

    // Ángulo del motor (donde está el AS5600)
    double angle_motor_deg = (raw * 360.0) / 4096.0;

    // Pasar a ángulo de salida teniendo en cuenta la reducción
    double angle = angle_motor_deg / EM_REDUCCION_1;
    double angle_rad = angle * (PI / 180.0);


    unsigned long dt = t_us - em_prevTime1_us;
    double vel = 0.0;
    if (dt > 0) {
        double dAng = angle - em_prevAngle1_deg;
        if (dAng > 180.0)       dAng -= 360.0;
        else if (dAng < -180.0) dAng += 360.0;
        vel = (dAng * 1e6) / (double)dt;
    }

    em_prevAngle1_deg = angle;
    em_prevTime1_us   = t_us;

    /*Serial.print(F("M1,"));
    Serial.print(angle, 3);  Serial.print(F(","));
    Serial.print(vel,   3);  Serial.print(F(","));
    Serial.println(raw);*/
    // Serial.println(angle_rad, 6);
    return angle_rad; 

}

// Imprime datos del encoder 2 (incremental) en CSV:
// M2,angle_deg,vel_deg_s,counts
 void EM_printEncoder2() {
  if (!em_init2) {
    em_prevTime2_us = micros();
    em_prevCounts2  = 0;
    em_init2        = true;
  }

  unsigned long t_us = micros();

  noInterrupts();
  long c = em_counts2;
  interrupts();

  double angle = ((double)c / (double)EM_CPR_eff_2) * (360.0 / EM_REDUCCION_2);
  double angle_rad = angle * (PI / 180.0);

  unsigned long dt = t_us - em_prevTime2_us;
  double vel = 0.0;
  if (dt > 0) {
    long dc = c - em_prevCounts2;
    double dt_s = (double)dt / 1e6;
    vel = (double)dc *
          (360.0 / ((double)EM_CPR_eff_2 * EM_REDUCCION_2)) /
          dt_s;
  }

  em_prevCounts2  = c;
  em_prevTime2_us = t_us;

  /* Serial.print(F("M2,"));
  Serial.print(angle, 3);  Serial.print(F(","));
  Serial.print(vel,   3);  Serial.print(F(","));
  Serial.println(c);*/ 
  Serial.println(angle_rad, 6);
}

#endif // ENCODERSMOTORES_H