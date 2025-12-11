/**
 * @file main.c
 * @brief Punto de entrada de la aplicación SumoBOT.
 * @details Orquestador principal del sistema. Inicializa los subsistemas y
 * ejecuta el bucle de control (o las pruebas de validación).
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

// Inclusiones del Proyecto
#include "include/RobotConfig.h"
#include "include/motor.h"

/**
 * @brief Inicializa periféricos básicos del sistema (LEDs, Debug).
 */
static void System_Init(void) {
    // Inicializar LED de Estado (PF1)
    SysCtlPeripheralEnable(LED_PERIPH);
    while(!SysCtlPeripheralReady(LED_PERIPH));
    GPIOPinTypeGPIOOutput(LED_BASE, LED_RED_PIN);
}

int main(void) {
    // --- 1. SETUP ---
    Motor_Init();   // Configura Clock 40MHz y Motores
    System_Init();  // Configura LED indicador

    // Señal visual de arranque (Boot check)
    LED_RED_ON();
    SysCtlDelay(SYSTEM_CLOCK_HZ / 10); // ~300ms
    LED_RED_OFF();

    // --- 2. LOOP INFINITO ---
    while(1) {
        // [MODO TEST] Validación de movimientos básicos
        
        // 1. Avance Frontal (Ataque)
        // Nota: Ajustar signos según polaridad física de los motores
        Motor_SetSpeed(80, -80); 
        LED_RED_ON();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 3); // ~1s

        // 2. Parada (Wait)
        Motor_Stop();
        LED_RED_OFF();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 3);

        // 3. Giro (Search/Spin)
        Motor_SetSpeed(60, 60);
        LED_RED_ON();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 3);
        
        // 4. Parada
        Motor_Stop();
        LED_RED_OFF();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 3);
    }
}