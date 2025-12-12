/**
 * @file main.c
 * @brief Punto de entrada SumoBOT - Test de Calibración de Giro.
 * @details Implementa una Máquina de Estados Finita (FSM) en el bucle principal
 * para verificar la precisión del giro de 60º utilizando odometría no bloqueante.
 * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.5.0 (Test Mode)
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h" // Necesario para IntMasterEnable()

// HAL Layers
#include "include/RobotConfig.h"    // Pines y Constantes
#include "include/motor.h"          // Control de Ruedas
#include "include/sensors.h"        // Vista y Tacto
#include "include/timer.h"          // SysTick
#include "include/odometry.h"       // <--- NUESTRA NUEVA LIBRERÍA

// --- DEFINICIÓN DE ESTADOS DEL TEST ---
typedef enum {
    TEST_STATE_INIT_WAIT,   // Espera inicial de seguridad (2s)
    TEST_STATE_START_TURN,  // Configura el giro
    TEST_STATE_TURNING,     // Gira y monitorea el ángulo
    TEST_STATE_FINISHED     // Parada final y validación visual
} TestState_t;

// Configuración manual del LED (Feedback visual)
void LED_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

// Helper para colores (R=2, B=4, G=8 en Tiva Launchpad)
void LED_Set(uint8_t pins) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, pins);
}

int main(void) {
    
    // --- 1. SETUP SYSTEM (CRÍTICO) ---
    // Configurar el Reloj a 40 MHz (PLL / 5)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Inicialización de periféricos
    Motor_Init();       // PWM
    LED_Init();         // Debug LEDs
    Timer_Init();       // SysTick Millis
    Sensors_Init();     // ADC / GPIOs
    Odometry_Init();    // Interrupciones PC5
    
    // ¡IMPORTANTE!: Habilitar interrupciones globales para que el contador funcione
    IntMasterEnable(); 

    // --- VARIABLES DE ESTADO ---
    TestState_t currentState = TEST_STATE_INIT_WAIT;
    float currentAngle = 0.0f;
    
    // Led Rojo: Sistema encendido, esperando
    LED_Set(GPIO_PIN_1); 

    // --- 2. LOOP INFINITO (NON-BLOCKING) ---
    while(1) {
        
        switch (currentState) {
            
            case TEST_STATE_INIT_WAIT:
                // Esperamos 2000ms para soltar el robot en el suelo
                if (Timer_GetMillis() > 2000) {
                    currentState = TEST_STATE_START_TURN;
                }
                break;

            case TEST_STATE_START_TURN:
                // 1. Reseteamos la odometría a 0
                Odometry_Reset();
                
                // 2. Iniciamos motores (Giro Izquierda sobre su eje)
                // Velocidad moderada para no derrapar y perder cuentas
                Motor_SetSpeed(-50, 50); 
                
                // Feedback: Led Azul (Girando)
                LED_Set(GPIO_PIN_2);
                
                currentState = TEST_STATE_TURNING;
                break;

            case TEST_STATE_TURNING:
                // Consultamos el ángulo actual (Operación rápida, no bloquea)
                currentAngle = Odometry_GetAngle();

                // CONDICIÓN DE SALIDA: ¿Llegamos a 60 grados?
                if (currentAngle >= 60.0f) {
                    Motor_Stop();
                    currentState = TEST_STATE_FINISHED;
                }

                // Aquí podrías añadir un "failsafe":
                // if (currentAngle > 360.0f) { Motor_Stop(); ... } // Algo va mal
                break;

            case TEST_STATE_FINISHED:
                // Estado final: Motores parados.
                Motor_Stop();
                
                // Feedback: Led Verde (Terminado)
                LED_Set(GPIO_PIN_3);
                
                // El robot se queda aquí para que puedas medir el ángulo real
                break;
        }
        
        // 3. Estabilización del ciclo (100Hz aprox)
        // Permite que la CPU respire y reduce ruido en sensores
        Timer_WaitMillis(10); 
    }
}