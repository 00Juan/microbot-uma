/**
 * @file sensors.h
 * @brief Driver de alto nivel para la adquisición de datos del entorno.
 * @details Gestiona la lectura de ADCs (Sharp, Línea) y GPIOs (Bumpers)
 * abstrayendo la complejidad del hardware y los secuenciadores.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Estructura de datos que contiene el estado completo del entorno.
 * Útil para pasar el estado a la lógica de decisión en un solo bloque.
 */
typedef struct
{
    uint32_t rawSharp;    // Valor ADC (0-4095)
    uint32_t rawLine;     // Valor ADC (0-4095)
    bool isBumperLeft;    // true = presionado
    bool isBumperRight;   // true = presionado
    bool isEnemyDetected; // true = rawSharp > Threshold
    bool isLineDetected;  // true = rawLine < Threshold (blanco refleja) o > Threshold (según lógica)
} SensorData_t;

/**
 * @brief Inicializa los periféricos de sensores (ADC0 y GPIOs).
 * @details Configura el Secuenciador 1 (SS1) para leer Sharp y Línea simultáneamente
 * y configura los pines de los bumpers con Pull-Up interno.
 */
void Sensors_Init(void);

/**
 * @brief Dispara una lectura y actualiza los valores internos.
 * @note Esta función debe llamarse al principio de cada ciclo del bucle principal
 * o en la interrupción del SysTick.
 */
void Sensors_Update(void);

/**
 * @brief Obtiene la última lectura procesada.
 * @return Copia de la estructura SensorData_t con los datos más recientes.
 */
SensorData_t Sensors_GetData(void);

// --- Getters Individuales (Helpers rápidos) ---

/**
 * @brief Verifica si el enemigo está en rango de ataque.
 * @return true si rawSharp > Umbral definido en RobotConfig.h
 */
bool Sensors_IsEnemyClose(void);

/**
 * @brief Verifica si estamos sobre la línea blanca (Peligro).
 * @return true si el sensor detecta blanco.
 */
bool Sensors_IsLineDetected(void);

#endif // SENSORS_H
