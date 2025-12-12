/**
 * @file odometry.h
 * @brief Interfaz de Odometría (Single-Encoder) para SumoBOT.
 * @details Módulo encargado de convertir los pulsos del sensor CNY70 (PC5)
 * en magnitudes físicas (grados y centímetros) de forma no bloqueante.
 * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 2.0.0 (Refactor Non-Blocking)
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Inicializa el hardware del Encoder (PC5).
 * @details Configura el pin como entrada con interrupción en ambos flancos 
 * (o flanco de bajada según sensibilidad) y habilita el reloj del puerto.
 */
void Odometry_Init(void);

/**
 * @brief Resetea el contador de ticks internos.
 * @details Debe llamarse justo antes de iniciar una maniobra controlada (giro o avance)
 * para establecer el punto cero.
 */
void Odometry_Reset(void);

/**
 * @brief Calcula los grados girados estimados desde el último Reset.
 * @details Asume una rotación "in-place" (giro sobre el propio eje), donde
 * las ruedas giran en sentidos opuestos a la misma velocidad.
 * Utiliza las constantes físicas definidas en RobotConfig.h.
 * * @return float Ángulo en grados (siempre positivo, magnitud del giro).
 */
float Odometry_GetAngle(void);

/**
 * @brief Obtiene la cuenta cruda de ticks.
 * @return uint32_t Número de cambios de estado detectados.
 */
uint32_t Odometry_GetTicks(void);

#endif // ODOMETRY_H