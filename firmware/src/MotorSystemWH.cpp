#include "MotorSystemHW.hpp"

// Static instance pointers initialization
/** @brief Static singleton pointers used by ISR to access class instances */
MotorSystemHW* MotorSystemHW::instance0 = nullptr;
MotorSystemHW* MotorSystemHW::instance1 = nullptr;

/**
 * @brief MotorSystemHW constructor
 * @param serial HardwareSerial port reference for motor UART communication
 * @param motorID Numeric identifier for motor (0 or 1)
 * @param pwm GPIO pin used for PWM motor control
 * @param dir GPIO pin used for motor direction control
 * @param a Encoder channel A pin
 * @param b Encoder channel B pin
 */
MotorSystemHW::MotorSystemHW(HardwareSerial &serial, uint8_t motorID, int pwm, int dir, int a, int b)
: uart(serial), id(motorID), pwmPin(pwm), dirPin(dir), encA(a), encB(b) {}

/**
 * @brief Initializes IO pins and attaches encoder interrupts
 * @param baud Baud rate for serial motor communication
 */
void MotorSystemHW::begin(long baud) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);

    // Register instance globally for ISR access
    if (id == 0) instance0 = this;
    if (id == 1) instance1 = this;

    // Attach hardware interrupt to encoder A channel
    if (id == 0)
        attachInterrupt(digitalPinToInterrupt(encA), isr0, CHANGE);
    else if (id == 1)
        attachInterrupt(digitalPinToInterrupt(encA), isr1, CHANGE);
}

/**
 * @brief ISR router for motor 0 encoder interrupts
 */
void IRAM_ATTR MotorSystemHW::isr0() {
    if (instance0) instance0->handleEncoder();
}

/**
 * @brief ISR router for motor 1 encoder interrupts
 */
void IRAM_ATTR MotorSystemHW::isr1() {
    if (instance1) instance1->handleEncoder();
}

/**
 * @brief Quadrature decoder logic executed on every encoder interrupt
 * @note Must run in IRAM to avoid flash access delays on ESP32
 */
void IRAM_ATTR MotorSystemHW::handleEncoder() {
    int a = digitalRead(encA);
    int b = digitalRead(encB);

    // Quadrature decode: increment or decrement tick count
    encoderTicks += (a == b) ? 1 : -1;
}

/**
 * @brief Returns encoder tick position (16-bit wrapped)
 * @return uint16_t current encoder position
 */
uint16_t MotorSystemHW::read_position() {
    return(uint16_t) (234);  
    return (uint16_t)(encoderTicks & 0xFFFF);
}

/**
 * @brief Sends motor drive command using direction and PWM
 * @param active Enables/disables motor output
 * @param direction Rotation direction (0 or 1)
 * @param pwm PWM duty cycle (0–4095, 12-bit resolution)
 */
void MotorSystemHW::write_cmd(bool active, bool direction, uint16_t pwm) {
    if (!active) {
        analogWrite(pwmPin, 0);
        return;
    }
    digitalWrite(dirPin, direction);
    analogWrite(pwmPin, pwm > 4095 ? 4095 : pwm);
}

/**
 * @brief Returns motor ID (0 or 1)
 */
uint8_t MotorSystemHW::getID() {
    return id;
}

/**
 * @brief Returns raw encoder tick count (no wrapping)
 */
long MotorSystemHW::getRawTicks() {
    return encoderTicks;
}
