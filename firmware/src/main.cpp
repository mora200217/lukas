#include <Arduino.h>
#include "MotorSystemHW.hpp"

// === PIN SETUP (change as needed) ===
#define M1_PWM  25
#define M1_DIR  26
#define M1_ENCA 32
#define M1_ENCB 33

#define M2_PWM  27
#define M2_DIR  14
#define M2_ENCA 34
#define M2_ENCB 35

MotorSystemHW motor1(Serial, 0, M1_PWM, M1_DIR, M1_ENCA, M1_ENCB);
MotorSystemHW motor2(Serial, 1, M2_PWM, M2_DIR, M2_ENCA, M2_ENCB);

enum State { STANDBY, READ, MOVE_AND_READ };
State state = STANDBY;

void printBanner() {
    Serial.println("=== ESP32 Motor Controller ===");
    Serial.print("Board: ESP32\n");
    Serial.print("Group: Lukas\n");
    Serial.print("Chip:  ");
    Serial.println(ESP.getChipModel());
    Serial.print("Cores: ");
    Serial.println(ESP.getChipCores());
    Serial.print("Rev:   ");
    Serial.println(ESP.getChipRevision());
    Serial.println("=============================");
}

// Decode 16-bit command: 1id | 1active | 1dir | 12pwm | 1null
void decodeAndCommand(uint16_t cmd) {
    uint8_t  id     = (cmd >> 15) & 0x01;
    uint8_t  active = (cmd >> 14) & 0x01;
    uint8_t  dir    = (cmd >> 13) & 0x01;
    uint16_t pwm    = (cmd >> 1)  & 0x0FFF;  // 12 bits

    if (id == 0)
        motor1.write_cmd(active, dir, pwm);
    else
        motor2.write_cmd(active, dir, pwm);
}


void printBoardInfo() {
    Serial.println("\n--- Board Info ---");
    Serial.printf("Model: %s\n", ESP.getChipModel());
    Serial.printf("Cores: %d\n", ESP.getChipCores());
    Serial.printf("Rev: %d\n", ESP.getChipRevision());
    Serial.println("--- Motor Info ---");
    Serial.printf("M1 ID: %d  Ticks: %ld\n", motor1.getID(), motor1.getRawTicks());
    Serial.printf("M2 ID: %d  Ticks: %ld\n\n", motor2.getID(), motor2.getRawTicks());
}

void setup() {
    Serial.begin(115200);

    motor1.begin();
    motor2.begin();

    printBanner();
    delay(1000);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 'i') printBoardInfo();
        else if (c == 's') state = STANDBY;
        else if (c == 'r') state = READ;
        else if (c == 'm') state = MOVE_AND_READ;
    }

    switch(state) {
        default: 
        case STANDBY:
            motor1.write_cmd(false, 0, 0);
            motor2.write_cmd(false, 0, 0);
            break;

      // ======= READ STATE ====================================
        case READ: {
              Serial.write(124); 
              delay(10); 
        break; 
            /*
            uint16_t p1 = motor1.read_position();
            uint16_t p2 = motor2.read_position();

            Serial.write((p1 >> 8) & 0xFF);
            Serial.write(p1 & 0xFF);
            Serial.write((p2 >> 8) & 0xFF);
            Serial.write(p2 & 0xFF);
            delay(10);
            break;*/
        }

        case MOVE_AND_READ: {
            if (Serial.available() >= 2) {
                uint16_t cmd = (Serial.read() << 8);
                cmd |= Serial.read();
                decodeAndCommand(cmd);
            }

            uint16_t p1 = motor1.read_position();
            uint16_t p2 = motor2.read_position();

            Serial.write((p1 >> 8) & 0xFF);
            Serial.write(p1 & 0xFF);
            Serial.write((p2 >> 8) & 0xFF);
            Serial.write(p2 & 0xFF);
            delay(10);
            break;
        }
    }
}
