/**
 * @file odometry.c
 * @brief Implementación del driver de Odometría Monocanal.
 * @details Gestiona la ISR del puerto C para contar pulsos del CNY70
 * y realiza los cálculos trigonométricos para estimar posición.
 * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 2.0.0
 */

#include "include/odometry.h"
#include "include/RobotConfig.h" // NECESARIO: Debe tener CM_PER_TICK y WHEEL_BASE_CM
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include <math.h>

// Variable Volátil: Crítico porque se modifica dentro de una ISR
static volatile uint32_t g_ticks_count = 0;

/**
 * @brief Manejador de Interrupciones del Puerto C (PC5).
 * @note Esta función debe estar registrada en el vector de interrupciones (startup_gcc.c).
 */
void GPIOC_IntHandler(void) {
    // 1. Limpiar la bandera de interrupción (Vital para no entrar en bucle infinito)
    GPIOIntClear(ENCODER_GPIO_BASE, ENCODER_PIN);
    
    // 2. Incrementar contador atómico
    g_ticks_count++;
}

void Odometry_Init(void) {
    // Activar reloj del puerto (definido en RobotConfig.h)
    SysCtlPeripheralEnable(ENCODER_GPIO_PERIPH);
    while(!SysCtlPeripheralReady(ENCODER_GPIO_PERIPH));

    // Configurar PC5 como entrada digital
    GPIOPinTypeGPIOInput(ENCODER_GPIO_BASE, ENCODER_PIN);

    // Configurar interrupción: Ambos flancos da mayor resolución (si el sensor es limpio)
    // Si hay mucho ruido, cambiar a GPIO_FALLING_EDGE
    GPIOIntTypeSet(ENCODER_GPIO_BASE, ENCODER_PIN, GPIO_BOTH_EDGES);

    // Habilitar la interrupción específica del pin y del puerto
    GPIOIntEnable(ENCODER_GPIO_BASE, ENCODER_PIN);
    IntEnable(ENCODER_INT_ID); 
}

void Odometry_Reset(void) {
    // Sección crítica: Podríamos deshabilitar ints momentáneamente si tememos corrupción
    g_ticks_count = 0;
}

uint32_t Odometry_GetTicks(void) {
    return g_ticks_count;
}

float Odometry_GetAngle(void) {
    /* TEORÍA DE GIRO DIFERENCIAL:
       El robot gira sobre un círculo cuyo diámetro es la distancia entre ruedas (WHEEL_BASE).
       
       1. Distancia recorrida por la rueda (Arco externo)
          S = ticks * CM_PER_TICK
       
       2. Relación Arco-Ángulo (en radianes)
          S = radio_giro * theta_rad
          radio_giro = WHEEL_BASE_CM / 2.0
       
       3. Despejando el ángulo en grados
          theta_deg = (S / radio_giro) * (180 / PI)
    */

    float distance_wheel_cm = (float)g_ticks_count * CM_PER_TICK;
    float radius_turn_cm = WHEEL_BASE_CM / 2.0f;

    // Evitar división por cero si la configuración está mal
    if (radius_turn_cm < 0.1f) return 0.0f;

    float angle_rad = distance_wheel_cm / radius_turn_cm;
    float angle_deg = angle_rad * (180.0f / 3.14159265f);
    
    return angle_deg;
}