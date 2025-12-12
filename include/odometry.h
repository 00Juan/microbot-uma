/**
 * @file odometry.h
 * @brief Interfaz de navegación basada en Encoders Ópticos (CNY70).
 * @details Permite realizar movimientos deterministas (avanzar X cm, girar Y grados)
 * utilizando contadores de pulsos mediante interrupciones GPIO.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0 (Experimental)
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Inicializa las interrupciones GPIO para los encoders.
 * @details Configura PC5 y PC6 como entradas con interrupción en ambos flancos
 * (subida y bajada) para maximizar la resolución de las franjas blanco/negro.
 */
void Odometry_Init(void);

/**
 * @brief Desplaza el robot una distancia lineal específica.
 * @details Función BLOQUEANTE. El robot avanzará o retrocederá hasta que el promedio
 * de los encoders alcance la distancia objetivo.
 * * @param cm Distancia en centímetros.
 * - Positivo: Avanzar.
 * - Negativo: Retroceder.
 */
void mover_robot(float cm);

/**
 * @brief Gira el robot un ángulo específico sobre su propio eje.
 * @details Función BLOQUEANTE. Mueve las ruedas en sentido contrario hasta completar el arco.
 * * @param deg Grados de giro.
 * - Positivo: Giro horario (Derecha).
 * - Negativo: Giro anti-horario (Izquierda).
 */
void girar_robot(float deg);

#endif // ODOMETRY_H