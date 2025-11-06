#pragma once
#include <Arduino.h>

class MotorSystemHW {
private:
    HardwareSerial &uart;
    uint8_t id;
    int pwmPin, dirPin, encA, encB;
    volatile long encoderTicks = 0;

public:
    MotorSystemHW(HardwareSerial &serial, uint8_t motorID, int pwm, int dir, int a, int b);
    void begin(long baud = 115200);
    uint16_t read_position();
    void write_cmd(bool active, bool direction, uint16_t pwm);
    uint8_t getID();
    long getRawTicks();

    // === Interrupt handler (instance) ===
    void handleEncoder();

    // === Static ISR trampolines ===
    static void IRAM_ATTR isr0();
    static void IRAM_ATTR isr1();

    // Store pointers to active instances
    static MotorSystemHW* instance0;
    static MotorSystemHW* instance1;
};
