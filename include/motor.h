/**
 * @file motor.h
 * @brief Controlador de alto nivel para motores (Servos Trucados).
 * @details Interfaz pública para el control de movimiento y velocidad
 * mediante PWM en la arquitectura Tiva C.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Inicializa el subsistema de motores.
 * @details Configura el reloj del sistema (si es necesario), habilita los periféricos PWM y GPIO,
 * y establece la configuración inicial de los generadores PWM según RobotConfig.h.
 */
void Motor_Init(void);

/**
 * @brief Controla la velocidad y dirección de los motores de forma diferencial.
 * * @param speed_izq Velocidad Motor Izquierdo en porcentaje [-100 a 100].
 * @param speed_der Velocidad Motor Derecho en porcentaje [-100 a 100].
 * * @note
 * - 0: Parada (Punto muerto calibrado).
 * - Positivo: Avance (Sentido horario/CW).
 * - Negativo: Retroceso (Sentido anti-horario/CCW).
 * - Los valores fuera de rango se limitan automáticamente (clamping).
 */
void Motor_SetSpeed(int8_t speed_izq, int8_t speed_der);

/**
 * @brief Detiene ambos motores inmediatamente.
 * @details Fuerza la señal PWM al valor de 'STOP' definido en la calibración.
 * Útil para frenado de emergencia o finalización de combate.
 */
void Motor_Stop(void);

#endif // MOTOR_H