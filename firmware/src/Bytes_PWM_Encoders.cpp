/*
  ESP32 - Control 2 motores con BTS7960 (2 PWM each)
  - Motor 1: AS5600 (I2C)   -> lectura multi-vuelta robusta (rollover handling)
  - Motor 2: Encoder incremental (ISR on A rising)
  - Telemetría TX (a PC/ROS2): frame binario
        [0xFF,
         enc1_b3, enc1_b2, enc1_b1, enc1_b0,
         enc2_b3, enc2_b2, enc2_b1, enc2_b0]
    * encX = int32_t en milésimas de grado (° * 1000)
  - Comandos RX (desde ROS2): [0xFF, 0xFA/0xFB, PWM_H, PWM_L]
      * 0xFA -> motor 1
      * 0xFB -> motor 2
      * PWM = int16 en centésimas de %, ej: 2500 -> +25.00 %
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/ledc.h"

// ==================== PINS MOTOR 1 (Maxon - AS5600) ====================
#define M1_RPWM_PIN    25
#define M1_LPWM_PIN    26
#define M1_R_EN_PIN    27
#define M1_L_EN_PIN    14
#define AS5600_SDA_PIN 21
#define AS5600_SCL_PIN 22
#define AS5600_ADDR    0x36
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D

// ==================== PINS MOTOR 2 (Faulhaber - incremental encoder) ====================
#define M2_RPWM_PIN    16
#define M2_LPWM_PIN    17
#define M2_R_EN_PIN    2
#define M2_L_EN_PIN    4
#define ENC2_A_PIN     32
#define ENC2_B_PIN     33

// ==================== PWM / LEDC ====================
#define REQUESTED_PWM_FREQ 1000UL  // 1 kHz
#define APB_FREQ 80000000UL

static const ledc_mode_t  LEDC_MODE  = LEDC_HIGH_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
// canales: 0/1 -> motor1 RPWM/LPWM ; 2/3 -> motor2 RPWM/LPWM
static const ledc_channel_t LEDC_CH_M1_R = LEDC_CHANNEL_0;
static const ledc_channel_t LEDC_CH_M1_L = LEDC_CHANNEL_1;
static const ledc_channel_t LEDC_CH_M2_R = LEDC_CHANNEL_2;
static const ledc_channel_t LEDC_CH_M2_L = LEDC_CHANNEL_3;

static uint32_t pwmMax = 0;

// ==================== TRANSMISIONES y CORRECCIONES ====================
const double TRANSMISSION_M1 = (26.0 / 7.0) * 60.0;   // ≈ 222.857142857
const double CORRECTION_M1   = 1.05;                  // factor de linealidad (M1)
const double TRANSMISSION_M2 = (141.0 / 26.0) * 2.0;  // ≈ 10.846153846
const double CORRECTION_M2   = 1.36;                  // factor de linealidad (M2)

// ==================== ENCODER 1 (AS5600) STATE ====================
static bool          enc1_init        = false;
static int           enc1_prevRaw     = 0;     // última lectura cruda 0..4095
static int32_t       enc1_rawMulti    = 0;     // cuentas acumuladas multi-vuelta
static unsigned long enc1_prevTime_us = 0;

#define ENC1_DEADBAND 0 // usar si quieres descartar diffs pequeños

// ==================== ENCODER 2 (incremental) STATE ====================
#define PPR_2        500
#define DECODE_X_2   1
#define CPR_eff_2    (PPR_2 * DECODE_X_2)
#define REDUCCION_2  TRANSMISSION_M2

static volatile long counts2      = 0;
static volatile bool em_dir2_ccw  = true;
static bool          em_init2     = false;

// ==================== CONTROL / TIMING / TELEMETRÍA ====================
unsigned long previousMicros     = 0;
unsigned long Ts                 = 1000;   // 1 ms base loop (1000 us)
unsigned long programStartTime   = 0;

// ==================== PWM / CONTROL VARS ====================
float   U1 = 0.0f;   // comando % motor 1
float   U2 = 0.0f;   // comando % motor 2
int32_t rpwm1 = 0, lpwm1 = 0, rpwm2 = 0, lpwm2 = 0;

// ==================== PROTOTIPOS ====================
static void   configPWM();
static void   applyPWM(int motor, float percentage);
static int    readAS5600Raw();
static void   IRAM_ATTR isr_enc2_a_rising();
static double updateAngle1Deg();           // multi-vuelta, salida (motor->salida) en grados
static double getAngle2Deg_internal();     // salida en grados
void          sendSerialData();
void          processSerialRxFSM();
void          enviarFrame(int32_t enc1, int32_t enc2);

// ==================== IMPLEMENTACIÓN ====================

static void configPWM() {
  double ratio   = (double)APB_FREQ / (double)REQUESTED_PWM_FREQ;
  double bits_d  = floor(log2(ratio));
  int    maxBits = (int)bits_d;
  if (maxBits > 16) maxBits = 16;
  if (maxBits < 1)  maxBits = 1;
  int pwmResolution = maxBits;
  pwmMax = ((1UL << pwmResolution) - 1UL);

  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode      = LEDC_MODE;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)pwmResolution;
  ledc_timer.timer_num       = LEDC_TIMER;
  ledc_timer.freq_hz         = REQUESTED_PWM_FREQ;
  ledc_timer.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ch = {};
  ch.speed_mode = LEDC_MODE;
  ch.timer_sel  = LEDC_TIMER;
  ch.intr_type  = LEDC_INTR_DISABLE;
  ch.hpoint     = 0;
  ch.duty       = 0;

  ch.gpio_num = M1_RPWM_PIN; ch.channel = LEDC_CH_M1_R;
  ledc_channel_config(&ch);

  ch.gpio_num = M1_LPWM_PIN; ch.channel = LEDC_CH_M1_L;
  ledc_channel_config(&ch);

  ch.gpio_num = M2_RPWM_PIN; ch.channel = LEDC_CH_M2_R;
  ledc_channel_config(&ch);

  ch.gpio_num = M2_LPWM_PIN; ch.channel = LEDC_CH_M2_L;
  ledc_channel_config(&ch);

  // asegurar duties iniciales 0
  ledc_set_duty(LEDC_MODE, LEDC_CH_M1_R, 0); ledc_update_duty(LEDC_MODE, LEDC_CH_M1_R);
  ledc_set_duty(LEDC_MODE, LEDC_CH_M1_L, 0); ledc_update_duty(LEDC_MODE, LEDC_CH_M1_L);
  ledc_set_duty(LEDC_MODE, LEDC_CH_M2_R, 0); ledc_update_duty(LEDC_MODE, LEDC_CH_M2_R);
  ledc_set_duty(LEDC_MODE, LEDC_CH_M2_L, 0); ledc_update_duty(LEDC_MODE, LEDC_CH_M2_L);
}

static void applyPWM(int motor, float percentage) {
  percentage = constrain(percentage, -100.0f, 100.0f);
  int32_t val = (int32_t)((fabs(percentage) / 100.0f) * (double)pwmMax + 0.5);
  if (val > (int32_t)pwmMax) val = (int32_t)pwmMax;
  if (val < 0)               val = 0;

  ledc_channel_t chR = (motor == 1) ? LEDC_CH_M1_R : LEDC_CH_M2_R;
  ledc_channel_t chL = (motor == 1) ? LEDC_CH_M1_L : LEDC_CH_M2_L;

  if (percentage >= 0.0f) {
    ledc_set_duty(LEDC_MODE, chR, (uint32_t)val);
    ledc_set_duty(LEDC_MODE, chL, 0);
    if (motor == 1) { rpwm1 = val; lpwm1 = 0; }
    else            { rpwm2 = val; lpwm2 = 0; }
  } else {
    ledc_set_duty(LEDC_MODE, chR, 0);
    ledc_set_duty(LEDC_MODE, chL, (uint32_t)val);
    if (motor == 1) { rpwm1 = 0; lpwm1 = val; }
    else            { rpwm2 = 0; lpwm2 = val; }
  }
  ledc_update_duty(LEDC_MODE, chR);
  ledc_update_duty(LEDC_MODE, chL);
}

// I2C read AS5600 raw (0..4095)
static int readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    int hi = Wire.read();
    int lo = Wire.read();
    return (((hi << 8) | lo) & 0x0FFF);
  }
  return 0;
}

static void IRAM_ATTR isr_enc2_a_rising() {
  int b = gpio_get_level((gpio_num_t)ENC2_B_PIN);
  if (!b) {
    counts2++;
    em_dir2_ccw = true;
  } else {
    counts2--;
    em_dir2_ccw = false;
  }
}

// ========== ANGULOS ==========
// Multi-vuelta AS5600 -> devuelve ángulo de salida [°]
static double updateAngle1Deg() {
  if (!enc1_init) {
    enc1_prevTime_us = micros();
    enc1_prevRaw     = readAS5600Raw();
    enc1_rawMulti    = 0;
    enc1_init        = true;
  }

  int raw = readAS5600Raw(); // 0..4095
  int16_t diff = (int16_t)raw - (int16_t)enc1_prevRaw;
  if (diff >  2048) diff -= 4096;
  if (diff < -2048) diff += 4096;

  if (ENC1_DEADBAND > 0) {
    if (diff > -ENC1_DEADBAND && diff < ENC1_DEADBAND) diff = 0;
  }

  enc1_rawMulti += diff;
  enc1_prevRaw   = raw;

  const double K_RAD_PER_COUNT = (2.0 * PI) / 4096.0;
  double angle_motor_rad       = (double)enc1_rawMulti * K_RAD_PER_COUNT;
  double angle_out_rad         = angle_motor_rad;  // sin otra reducción aquí
  double angle_out_deg_motor   = angle_out_rad * 180.0 / PI;

  double corrected_motor_deg   = angle_out_deg_motor * CORRECTION_M1;
  double output_deg            = corrected_motor_deg / TRANSMISSION_M1;

  return output_deg;
}

// Devuelve ángulo de salida motor 2 en grados
static double getAngle2Deg_internal() {
  noInterrupts();
  long c = counts2;
  interrupts();

  double angle_motor_deg = ((double)c / (double)CPR_eff_2) * 360.0;
  double output_deg      = (angle_motor_deg / TRANSMISSION_M2) * CORRECTION_M2;
  return output_deg;
}

// ==================== API / SETUP / LOOP ====================

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  configPWM();

  // EN pins
  pinMode(M1_R_EN_PIN, OUTPUT); pinMode(M1_L_EN_PIN, OUTPUT);
  digitalWrite(M1_R_EN_PIN, HIGH); digitalWrite(M1_L_EN_PIN, HIGH);
  pinMode(M2_R_EN_PIN, OUTPUT); pinMode(M2_L_EN_PIN, OUTPUT);
  digitalWrite(M2_R_EN_PIN, HIGH); digitalWrite(M2_L_EN_PIN, HIGH);

  // I2C
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.endTransmission();

  // Encoder incremental motor2
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), isr_enc2_a_rising, RISING);

  enc1_init        = false;
  em_init2         = false;
  programStartTime = micros();

  // arrancar motores en 0
  U1 = 0.0f; U2 = 0.0f;
  applyPWM(1, U1);
  applyPWM(2, U2);
}

void loop() {
  unsigned long now_us = micros();

  // Procesar comandos binarios entrantes (ROS2 -> ESP32)
  processSerialRxFSM();

  // base telemetría a Ts
  if (now_us - previousMicros >= Ts) {
    previousMicros = now_us;

    // aplicar PWMs actuales
    applyPWM(1, U1);
    applyPWM(2, U2);

    // enviar frame de encoders
    sendSerialData();
  }
}

// ==================== TELEMETRÍA (TX a ROS2) ====================
void enviarFrame(int32_t enc1, int32_t enc2) {
  Serial.write(0xFF);  // header

  // Encoder 1 (int32, big-endian)
  Serial.write((uint8_t)((enc1 >> 24) & 0xFF));
  Serial.write((uint8_t)((enc1 >> 16) & 0xFF));
  Serial.write((uint8_t)((enc1 >> 8)  & 0xFF));
  Serial.write((uint8_t)( enc1        & 0xFF));

  // Encoder 2 (int32, big-endian)
  Serial.write((uint8_t)((enc2 >> 24) & 0xFF));
  Serial.write((uint8_t)((enc2 >> 16) & 0xFF));
  Serial.write((uint8_t)((enc2 >> 8)  & 0xFF));
  Serial.write((uint8_t)( enc2        & 0xFF));
}

void sendSerialData() {
  // tiempo en ms (por si luego lo quieres usar)
  unsigned long t_ms = (micros() - programStartTime) / 1000;
  (void)t_ms; // evitar warning de variable no usada

  double s1 = updateAngle1Deg();        // sensor motor 1 (°)
  double s2 = getAngle2Deg_internal();  // sensor motor 2 (°)

  // Escalamos a milésimas de grado para no perder decimales
  int32_t enc1 = (int32_t)round(s1 * 1000.0);
  int32_t enc2 = (int32_t)round(s2 * 1000.0);

  enviarFrame(enc1, enc2);
}

// ==================== RX: MÁQUINA DE ESTADOS (ROS2 -> PWM) ====================

enum RxState {
  RX_WAIT_HEADER = 0,
  RX_WAIT_ID,
  RX_WAIT_PWM_H,
  RX_WAIT_PWM_L
};

void processSerialRxFSM() {
  static RxState state = RX_WAIT_HEADER;
  static int rx_id     = 0;
  static int pwm_hi    = 0;

  while (Serial.available() > 0) {
    int b = (int)Serial.read();

    switch (state) {
      case RX_WAIT_HEADER:
        if (b == 0xFF) {
          state = RX_WAIT_ID;
        }
        break;

      case RX_WAIT_ID:
        if (b == 0xFA || b == 0xFB) {
          rx_id = b;
          state = RX_WAIT_PWM_H;
        } else if (b == 0xFF) {
          state = RX_WAIT_ID;
        } else {
          state = RX_WAIT_HEADER;
        }
        break;

      case RX_WAIT_PWM_H:
        pwm_hi = b & 0xFF;
        state  = RX_WAIT_PWM_L;
        break;

      case RX_WAIT_PWM_L: {
        int pwm_lo = b & 0xFF;
        int16_t pwm_raw = (int16_t)((pwm_hi << 8) | pwm_lo);  // int16 firmado

        // PWM en centésimas de % -> convertir a [-100, 100]
        float pwm_pct = (float)pwm_raw / 100.0f;
        pwm_pct = constrain(pwm_pct, -100.0f, 100.0f);

        if (rx_id == 0xFA) {
          U1 = pwm_pct;     // comando motor 1
        } else if (rx_id == 0xFB) {
          U2 = pwm_pct;     // comando motor 2
        }

        // volver al estado inicial
        state = RX_WAIT_HEADER;
        break;
      }
    }
  }
}
