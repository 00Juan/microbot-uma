/**
 * @file main.c
 * @brief Punto de entrada SumoBOT.
 * @author Alejandro Moyano Crespillo (AleSMC)
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

// HAL Layers
#include "include/RobotConfig.h"    //Pines y Constantes
#include "include/motor.h"          // Control de Ruedas
#include "include/sensors.h"        // Vista y Tacto
#include "include/timer.h"
// Logic Layer
#include "include/strategy.h"       // Toma de decisiones

// Configuración manual del LED (si no está en otro lado)
void LED_Init(void) {
    SysCtlPeripheralEnable(LED_PERIPH);         // Activa el reloj del puerto F
    while(!SysCtlPeripheralReady(LED_PERIPH));  // Espera a que el hardware despierte
    GPIOPinTypeGPIOOutput(LED_BASE, LED_RED_PIN); // Configura PF1 como Salida
}

int main(void) {
    
    // --- 1. SETUP SYSTEM (CRÍTICO) ---
    // Configurar el Reloj a 40 MHz (PLL / 5)
    // Esto DEBE ser lo primero antes de tocar cualquier periférico.
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // 1. Motor_Init(); 
    // CRITICO: En tu implementación actual, esta función configura el 
    // Reloj del Sistema a 40MHz (SysCtlClockSet). 
    // Si movieras esto al final, todo lo demás fallaría porque el reloj estaría mal configurado.
    Motor_Init();

    // 2. LED_Init();
    LED_Init();

    // 3. Timer_Init();
    // IMPORTANTE: El SysTick se calcula basándose en la velocidad del reloj (40MHz).
    // Si Motor_Init no se hubiera ejecutado antes, el Timer contaría mal los milisegundos.
    Timer_Init(); 

    // 4. Sensors_Init();
    // Configura el ADC y los GPIOs de los bumpers.
    Sensors_Init();
    

    // --- 2. SETUP STRATEGY (Wait 5s) ---
    Strategy_Init();

    // --- 3. LOOP INFINITO ---
    while(1) {
        // 1. Ejecutar la lógica de combate
        Strategy_Run();
        
        // 2. Estabilización del ciclo
        Timer_WaitMillis(10); 
    }
}