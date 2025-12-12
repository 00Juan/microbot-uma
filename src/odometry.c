/**
 * @file odometry.c
 * @brief Driver de Odometría y Movimiento Preciso.
 * @details Utiliza interrupciones GPIO en PC5/PC6 para leer encoders monocanal.
 * @author Alejandro Moyano Crespillo (AleSMC)
 */

#include "include/odometry.h"
#include "include/RobotConfig.h"
#include "include/motor.h"
#include "include/timer.h" // Para timeouts de seguridad

#include <stdint.h>
#include <stdbool.h>
#include <math.h> // Para fabs()

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

// --- VARIABLES GLOBALES VOLÁTILES (Contadores) ---
static volatile uint32_t g_ticks_left = 0;
static volatile uint32_t g_ticks_right = 0;

// --- ISR HANDLER (Se ejecuta cada vez que cambia el color en el encoder) ---
void GPIOC_IntHandler(void) {
    // 1. Leer estado de interrupción para saber qué pin disparó
    uint32_t status = GPIOIntStatus(ENCODER_GPIO_BASE, true);
    
    // 2. Limpiar bandera (CRÍTICO: Si no, se queda colgado aquí)
    GPIOIntClear(ENCODER_GPIO_BASE, status);

    // 3. Incrementar contadores
    if (status & ENCODER_LEFT_PIN) {
        g_ticks_left++;
    }
    if (status & ENCODER_RIGHT_PIN) {
        g_ticks_right++;
    }
}

// --- INICIALIZACIÓN ---
void Odometry_Init(void) {
    SysCtlPeripheralEnable(ENCODER_GPIO_PERIPH);
    while(!SysCtlPeripheralReady(ENCODER_GPIO_PERIPH));

    // Configurar PC5 y PC6 como entrada
    GPIOPinTypeGPIOInput(ENCODER_GPIO_BASE, ENCODER_LEFT_PIN | ENCODER_RIGHT_PIN);

    // Configurar Interrupciones: Ambos flancos (Blanco->Negro y Negro->Blanco)
    GPIOIntTypeSet(ENCODER_GPIO_BASE, ENCODER_LEFT_PIN | ENCODER_RIGHT_PIN, GPIO_BOTH_EDGES);

    // Registrar Handler (Sustituye a hacerlo en el startup_ccs manualmente)
    GPIOIntRegister(ENCODER_GPIO_BASE, GPIOC_IntHandler);

    // Habilitar
    GPIOIntEnable(ENCODER_GPIO_BASE, ENCODER_LEFT_PIN | ENCODER_RIGHT_PIN);
}

// --- FUNCIONES DEL ENUNCIADO ---

/**
 * @brief Se desplaza 'c' centímetros (Adelante o Atrás).
 * @param cm Distancia en cm. Positivo=Adelante, Negativo=Atrás.
 */
void mover_robot(float cm) {
    // 1. Resetear contadores
    g_ticks_left = 0;
    g_ticks_right = 0;

    // 2. Calcular objetivo de ticks
    // Distancia absoluta (los encoders siempre suman positivo)
    uint32_t target_ticks = (uint32_t)(fabs(cm) / CM_PER_TICK);
    
    // 3. Mover motores
    if (cm > 0) {
        Motor_SetSpeed(60, -60); // Adelante (Velocidad media para precisión)
    } else {
        Motor_SetSpeed(-60, 60); // Atrás
    }

    // 4. Esperar hasta alcanzar la cuenta (Bloqueante, como pide la práctica)
    // Usamos el promedio de ambas ruedas para decidir
    while(1) {
        uint32_t average_ticks = (g_ticks_left + g_ticks_right) / 2;
        if (average_ticks >= target_ticks) {
            break; // Llegamos
        }
        
        // Timeout de seguridad opcional: Si el robot choca y las ruedas patinan
        // el encoder seguirá contando y creeremos que nos movemos.
        // En esta práctica simple, no controlamos el patinaje.
    }

    // 5. Frenar
    Motor_Stop();
    Timer_WaitMillis(100); // Estabilizar
}

/**
 * @brief Gira 'g' grados sobre su propio eje.
 * @param deg Grados. Positivo=Derecha(Horario), Negativo=Izquierda.
 */
void girar_robot(float deg) {
    // 1. Resetear contadores
    g_ticks_left = 0;
    g_ticks_right = 0;

    // 2. Matemáticas de Giro (Diferencial)
    // Longitud del arco que debe recorrer cada rueda para girar 'deg' grados
    // Arco = 2 * pi * Radio_Giro * (Angulo / 360)
    // En giro sobre eje, el Radio_Giro es la mitad del ancho del robot (WHEEL_BASE / 2)
    float arc_length = (M_PI * WHEEL_BASE_CM * fabs(deg)) / 360.0f;
    
    uint32_t target_ticks = (uint32_t)(arc_length / CM_PER_TICK);

    // 3. Mover motores (Giro sobre eje: uno avanza, otro retrocede)
    if (deg > 0) {
        // Derecha (CW): Izq Adelante, Der Atrás
        Motor_SetSpeed(60, 60); // Ajustar signos según tu hardware
    } else {
        // Izquierda (CCW): Izq Atrás, Der Adelante
        Motor_SetSpeed(-60, -60);
    }

    // 4. Esperar
    while(1) {
        uint32_t average_ticks = (g_ticks_left + g_ticks_right) / 2;
        if (average_ticks >= target_ticks) {
            break;
        }
    }

    // 5. Frenar
    Motor_Stop();
    Timer_WaitMillis(100);
}