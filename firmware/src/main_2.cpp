// #include <Arduino.h>
// #include "MotorSystemHW.hpp"
// #include "utils.hpp"

// // Pines de conexión  ===============================================
// #define M1_PWM  25  ///< Pin PWM del motor 1
// #define M1_DIR  26  ///< Pin dirección del motor 1
// #define M1_ENCA 32  ///< Encoder canal A motor 1
// #define M1_ENCB 33  ///< Encoder canal B motor 1

// #define M2_PWM  27  ///< Pin PWM del motor 2
// #define M2_DIR  14  ///< Pin dirección del motor 2
// #define M2_ENCA 34  ///< Encoder canal A motor 2
// #define M2_ENCB 35  ///< Encoder canal B motor 2

// /**
//  * @brief Decodifica un paquete de comando de 16 bits y gobierna el motor correspondiente.
//  * @param cmd Comando serializado de 16 bits con formato propietario.
//  */
// void decodeAndCommand(uint16_t cmd); 

// // Definición de Motores  ===========================================

// /**
//  * @brief Instancia del sistema de control para el motor 1.
//  */
// MotorSystemHW motor1(Serial, 0, M1_PWM, M1_DIR, M1_ENCA, M1_ENCB);

// /**
//  * @brief Instancia del sistema de control para el motor 2.
//  */
// MotorSystemHW motor2(Serial, 1, M2_PWM, M2_DIR, M2_ENCA, M2_ENCB);

// // Definición de Máquina de Estados =================================

// /**
//  * @enum State
//  * @brief Estados de la máquina de control principal.
//  * 
//  * @var STANDBY      Motores apagados
//  * @var READ         Solo transmite datos del sistema
//  * @var MOVE_AND_READ Recibe comandos de movimiento + transmite lecturas de encoders
//  */
// enum State { STANDBY, READ, MOVE_AND_READ };

// /**
//  * @brief Estado actual de la máquina de estados.
//  */
// State state = READ;

// /**
//  * @brief Configuración inicial del sistema (se ejecuta una vez).
//  */
// void setup() {
//     Serial.begin(115200);

//     motor1.begin();
//     motor2.begin();

//     printBanner();
//     delay(1000);
// }

// /**
//  * @brief Bucle principal del firmware. Maneja la FSM y la comunicación serial.
//  */
// void loop() {
//     // Captura comandos de control de estado desde UART
//     if (Serial.available()) {
//         char c = Serial.read();

//         if (c == 'i') printBoardInfo();     // Imprime información del hardware
//         else if (c == 's') state = STANDBY; // Detiene motores
//         else if (c == 'r') state = READ;    // Modo solo lectura
//         else if (c == 'm') state = MOVE_AND_READ; // Modo lectura + movimiento
//     }

//     switch(state) {
//         default: 
//         case STANDBY:
//             // Apaga ambos motores
//             motor1.write_cmd(false, 0, 0);
//             motor2.write_cmd(false, 0, 0);
//             break;

//         // ======= MODO SOLO LECTURA ========================================
//         case READ: {
//             // Envía continuamente un byte de test (234 = 0xEA)
//             Serial.write(234);
//             delay(10); 
//             break; 
//         }

//         // ======= MODO CONTROL + LECTURA ===================================
//         case MOVE_AND_READ: {

//             // Si hay 2 bytes disponibles, interpretar como comando de motor
//             if (Serial.available() >= 2) {
//                 uint16_t cmd = (Serial.read() << 8);
//                 cmd |= Serial.read();
//                 decodeAndCommand(cmd);
//             }

//             // Leer posición de encoders
//             uint16_t p1 = motor1.read_position();
//             uint16_t p2 = motor2.read_position();

//             // Enviar las posiciones por UART (big-endian, 4 bytes)
//             Serial.write((p1 >> 8) & 0xFF);
//             Serial.write(p1 & 0xFF);
//             Serial.write((p2 >> 8) & 0xFF);
//             Serial.write(p2 & 0xFF);
//             delay(10);
//             break;
//         }
//     }
// }

// /**
//  * @brief Decodifica un comando de 16 bits y envía la orden al motor indicado.
//  * 
//  * @details El frame de 16 bits se interpreta de la siguiente manera:
//  * ```
//  *  [15]    → ID del motor (0 = motor1, 1 = motor2)
//  *  [14]    → Activo (1 = habilitar, 0 = deshabilitar)
//  *  [13]    → Dirección (0 = sentido A, 1 = sentido B)
//  *  [12:1]  → PWM (12 bits, 0 a 4095)
//  *  [0]     → Reservado (no usado)
//  * ```
//  *
//  * @param cmd Comando empaquetado a 16 bits.
//  */
// void decodeAndCommand(uint16_t cmd) {
//     uint8_t  id     = (cmd >> 15) & 0x01;
//     uint8_t  active = (cmd >> 14) & 0x01;
//     uint8_t  dir    = (cmd >> 13) & 0x01;
//     uint16_t pwm    = (cmd >> 1)  & 0x0FFF;  // 12 bits

//     if (id == 0)  
//         motor1.write_cmd(active, dir, pwm);
//     else
//         motor2.write_cmd(active, dir, pwm);
// }
