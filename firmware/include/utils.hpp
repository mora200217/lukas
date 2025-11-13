#include <Arduino.h>
#include "MotorSystemHW.hpp"

#ifndef __UTILS_HPP
#define __UTILS_HPP

/**
 * @brief Imprime un banner de inicio con información básica del firmware y del chip ESP32.
 * @details Muestra el nombre del board, grupo, modelo del chip, número de núcleos y revisión.
 * @note Requiere que Serial ya haya sido inicializado.
 */
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


/**
 * @brief Imprime información detallada del microcontrolador ESP32.
 * @details Muestra el modelo, número de núcleos y revisión del chip.
 * @note Requiere Serial previamente inicializado.
 */
void printBoardInfo() {
    Serial.println("\n--- Board Info ---");
    Serial.printf("Model: %s\n", ESP.getChipModel());
    Serial.printf("Cores: %d\n", ESP.getChipCores());
    Serial.printf("Rev: %d\n", ESP.getChipRevision());
}

#endif
