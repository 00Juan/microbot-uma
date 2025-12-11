/**
 * @file timer.c
 * @brief Implementación del SysTick Timer.
 * @author Alejandro Moyano Crespillo (AleSMC)
 */

#include "include/timer.h"
#include "include/RobotConfig.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

// Variable volátil porque cambia dentro de una interrupción
static volatile uint32_t g_millisCounter = 0;

// --- INTERRUPT HANDLER (ISR) ---
// Esta función se llama automáticamente 1000 veces por segundo
void SysTick_Handler(void) {
    g_millisCounter++;
}

// --- API ---

void Timer_Init(void) {
    // Configurar SysTick para dispararse cada 1ms
    // Periodo = Clock / 1000
    SysTickPeriodSet(SYSTEM_CLOCK_HZ / 1000);
    
    // Habilitar interrupciones y el contador
    SysTickIntEnable();
    SysTickEnable();
}

uint32_t Millis(void) {
    // Lectura atómica (en micros de 32 bits es seguro leer directamente)
    return g_millisCounter;
}

void Timer_WaitMillis(uint32_t ms) {
    uint32_t start = g_millisCounter;
    while((g_millisCounter - start) < ms) {
        // Espera activa pero usando el reloj real
    }
}