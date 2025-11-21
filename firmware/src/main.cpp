/*
  ESP32 - Control 2 motores con BTS7960 (2 PWM each)
  - Motor 1: AS5600 (I2C) multi-vuelta
  - Motor 2: Encoder incremental (ISR)
  - Telemetría TX (a PC/ROS2): frame binario
        [0xFF, enc1_hi, enc1_lo, enc2_hi, enc2_lo]
      encX = int16 = milésimas de grado (° * 1000)
  - Comandos RX: [0xFF, 0xFA/0xFB, PWM_H, PWM_L]
      0xFA -> motor 1
      0xFB -> motor 2
      PWM = int16 = centésimas de % (2500 = 25.00%)
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/ledc.h"

// ==================== PINS MOTOR 1 (AS5600) ====================
#define M1_RPWM_PIN    25
#define M1_LPWM_PIN    26
#define M1_R_EN_PIN    27
#define M1_L_EN_PIN    14
#define AS5600_SDA_PIN 21
#define AS5600_SCL_PIN 22
#define AS5600_ADDR    0x36
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D

// ==================== PINS MOTOR 2 (incremental) ====================
#define M2_RPWM_PIN    16
#define M2_LPWM_PIN    17
#define M2_R_EN_PIN    2
#define M2_L_EN_PIN    4
#define ENC2_A_PIN     32
#define ENC2_B_PIN     33

// ==================== PWM ====================
#define REQUESTED_PWM_FREQ 1000UL
#define APB_FREQ 80000000UL

static const ledc_mode_t  LEDC_MODE  = LEDC_HIGH_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;

static const ledc_channel_t LEDC_CH_M1_R = LEDC_CHANNEL_0;
static const ledc_channel_t LEDC_CH_M1_L = LEDC_CHANNEL_1;
static const ledc_channel_t LEDC_CH_M2_R = LEDC_CHANNEL_2;
static const ledc_channel_t LEDC_CH_M2_L = LEDC_CHANNEL_3;

static uint32_t pwmMax = 0;

// ==================== TRANSMISIONES / CORRECCIONES ====================
const double TRANSMISSION_M1 = (26.0 / 7.0) * 60.0; 
const double CORRECTION_M1   = 1.05;

const double TRANSMISSION_M2 = (141.0 / 26.0) * 2.0;
const double CORRECTION_M2   = 1.36;

// ==================== ENCODER 1 (AS5600) ====================
static bool    enc1_init        = false;
static int     enc1_prevRaw     = 0;
static int32_t enc1_rawMulti    = 0;

// ==================== ENCODER 2 (incremental) ====================
#define PPR_2        500
#define DECODE_X_2   1
#define CPR_eff_2    (PPR_2 * DECODE_X_2)

static volatile long counts2   = 0;

// ==================== CONTROL ====================
unsigned long previousMicros = 0;
unsigned long Ts = 5000;   // 1ms Loop
unsigned long programStartTime = 0;

float U1 = 0.0f;
float U2 = 0.0f;

// ==================== PROTOTIPOS ====================
static void   configPWM();
static void   applyPWM(int motor, float percentage);
static int    readAS5600Raw();
static void   IRAM_ATTR isr_enc2_a_rising();
static double updateAngle1Deg();
static double getAngle2Deg_internal();
void          sendSerialData();
void          processSerialRxFSM();
void          enviarFrame(int16_t enc1, int16_t enc2);

// ==================== PWM CONFIG ====================

static void configPWM() {
  double ratio   = (double)APB_FREQ / (double)REQUESTED_PWM_FREQ;
  double bits_d  = floor(log2(ratio));
  int pwmResolution = (int)bits_d;
  if (pwmResolution > 16) pwmResolution = 16;
  if (pwmResolution < 1)  pwmResolution = 1;

  pwmMax = (1UL << pwmResolution) - 1UL;

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

  ch.gpio_num = M1_RPWM_PIN; ch.channel = LEDC_CH_M1_R; ledc_channel_config(&ch);
  ch.gpio_num = M1_LPWM_PIN; ch.channel = LEDC_CH_M1_L; ledc_channel_config(&ch);
  ch.gpio_num = M2_RPWM_PIN; ch.channel = LEDC_CH_M2_R; ledc_channel_config(&ch);
  ch.gpio_num = M2_LPWM_PIN; ch.channel = LEDC_CH_M2_L; ledc_channel_config(&ch);
}

static void applyPWM(int motor, float percentage) {
  percentage = constrain(percentage, -100.0f, 100.0f);
  int32_t val = (int32_t)((fabs(percentage) / 100.0f) * pwmMax + 0.5);

  ledc_channel_t chR = (motor == 1) ? LEDC_CH_M1_R : LEDC_CH_M2_R;
  ledc_channel_t chL = (motor == 1) ? LEDC_CH_M1_L : LEDC_CH_M2_L;

  if (percentage >= 0) {
    ledc_set_duty(LEDC_MODE, chR, val);
    ledc_set_duty(LEDC_MODE, chL, 0);
  } else {
    ledc_set_duty(LEDC_MODE, chR, 0);
    ledc_set_duty(LEDC_MODE, chL, val);
  }
  ledc_update_duty(LEDC_MODE, chR);
  ledc_update_duty(LEDC_MODE, chL);
}

// ==================== ENCODER 1 (AS5600) ====================

static int readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return enc1_prevRaw;

  int hi = Wire.read();
  int lo = Wire.read();

  return ((hi << 8) | lo) & 0x0FFF;
}

static double updateAngle1Deg() {
  if (!enc1_init) {
    enc1_prevRaw  = readAS5600Raw();
    enc1_rawMulti = 0;
    enc1_init     = true;
  }

  int raw = readAS5600Raw();
  int16_t diff = (int16_t)(raw - enc1_prevRaw);

  if (diff >  2048) diff -= 4096;
  if (diff < -2048) diff += 4096;

  enc1_rawMulti += diff;
  enc1_prevRaw   = raw;

  double angle_motor_rad     = enc1_rawMulti * (2.0 * PI / 4096.0);
  double angle_motor_deg     = angle_motor_rad * 180.0 / PI;
  double corrected_motor_deg = angle_motor_deg * CORRECTION_M1;
  double output_deg          = corrected_motor_deg / TRANSMISSION_M1;

  return output_deg;
}

// ==================== ENCODER 2 (incremental) ====================

static void IRAM_ATTR isr_enc2_a_rising() {
  int b = gpio_get_level((gpio_num_t)ENC2_B_PIN);
  if (!b) counts2++;
  else    counts2--;
}

static double getAngle2Deg_internal() {
  noInterrupts();
  long c = counts2;
  interrupts();

  double angle_motor_deg = ((double)c / CPR_eff_2) * 360.0;
  double output_deg = (angle_motor_deg / TRANSMISSION_M2) * CORRECTION_M2;

  return output_deg;
}

// ==================== SERIAL (TX) ====================

void enviarFrame(int16_t enc1, int16_t enc2) {
  Serial.write(0xFF);

  Serial.write((uint8_t)((enc1 >> 8) & 0xFF));
  Serial.write((uint8_t)(enc1 & 0xFF));

  Serial.write((uint8_t)((enc2 >> 8) & 0xFF));
  Serial.write((uint8_t)(enc2 & 0xFF));
}

void sendSerialData() {
  double s1 = updateAngle1Deg();
  double s2 = getAngle2Deg_internal();

  int16_t enc1 = (int16_t)round(s1 * 1000.0);
  int16_t enc2 = (int16_t)round(s2 * 1000.0);

  enviarFrame(enc1, enc2);
}

// ==================== SERIAL (RX) ====================

enum RxState { RX_WAIT_HEADER, RX_WAIT_ID, RX_WAIT_PWM_H, RX_WAIT_PWM_L };

void processSerialRxFSM() {
  static RxState state = RX_WAIT_HEADER;
  static int rx_id = 0;
  static int pwm_hi = 0;

  while (Serial.available() > 0) {
    int b = Serial.read();

    switch (state) {
      case RX_WAIT_HEADER:
        if (b == 0xFF) state = RX_WAIT_ID;
        break;

      case RX_WAIT_ID:
        if (b == 0xFA || b == 0xFB) {
          rx_id = b;
          state = RX_WAIT_PWM_H;
        } else state = RX_WAIT_HEADER;
        break;

      case RX_WAIT_PWM_H:
        pwm_hi = b;
        state = RX_WAIT_PWM_L;
        break;

      case RX_WAIT_PWM_L: {
        int pwm_lo = b;
        int16_t pwm_raw = (int16_t)((pwm_hi << 8) | pwm_lo);
        float pwm_pct = pwm_raw / 100.0f;

        if (rx_id == 0xFA) U1 = pwm_pct;
        if (rx_id == 0xFB) U2 = pwm_pct;

        state = RX_WAIT_HEADER;
      } break;
    }
  }
}

// ==================== SETUP / LOOP ====================

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  configPWM();

  pinMode(M1_R_EN_PIN, OUTPUT);
  pinMode(M1_L_EN_PIN, OUTPUT);
  pinMode(M2_R_EN_PIN, OUTPUT);
  pinMode(M2_L_EN_PIN, OUTPUT);
  digitalWrite(M1_R_EN_PIN, HIGH);
  digitalWrite(M1_L_EN_PIN, HIGH);
  digitalWrite(M2_R_EN_PIN, HIGH);
  digitalWrite(M2_L_EN_PIN, HIGH);

  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  Wire.setClock(400000);

  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), isr_enc2_a_rising, RISING);

  programStartTime = micros();
}

void loop() {
  unsigned long now_us = micros();

  processSerialRxFSM();

  if (now_us - previousMicros >= Ts) {
    previousMicros = now_us;

    applyPWM(1, U1);
    applyPWM(2, U2);

    sendSerialData();
  }
}
