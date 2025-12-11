/**
 * @file timer.h
 * @brief Control de tiempo del sistema (System Tick).
 * @author Alejandro Moyano Crespillo (AleSMC)
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Inicializa el SysTick para generar interrupciones cada 1ms.
 */
void Timer_Init(void);

/**
 * @brief Devuelve los milisegundos transcurridos desde el arranque.
 * @return uint32_t Tiempo en ms. (Se desborda cada 49 días, suficiente para Sumo).
 */
uint32_t Millis(void);

/**
 * @brief Espera bloqueante (Solo usar en inicialización, NUNCA en combate).
 */
void Timer_WaitMillis(uint32_t ms);

#endif // TIMER_H
