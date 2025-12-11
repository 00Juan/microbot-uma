/**
 * @file strategy.h
 * @brief Máquina de Estados Finita (FSM) para SumoBOT - Versión V1.0 (Reactiva).
 * @details Implementación de lógica directa: Sobrevivir > Atacar > Buscar.
 * NOTA: Esta versión utiliza retardos bloqueantes para maniobras de evasión.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#ifndef STRATEGY_H
#define STRATEGY_H

/**
 * @brief Rutina de arranque reglamentaria.
 * @details Ejecuta la espera de 5 segundos (Normativa) parpadeando el LED,
 * y prepara el sistema para el combate.
 */
void Strategy_Init(void);

/**
 * @brief Ejecuta un ciclo de la lógica de control.
 * @details Lee sensores, toma decisiones según prioridades y mueve motores.
 * Se debe llamar dentro del while(1) del main.
 */
void Strategy_Run(void);

#endif // STRATEGY_H
