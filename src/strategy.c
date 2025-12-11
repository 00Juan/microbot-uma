/**
 * @file strategy.c
 * @brief Lógica de Combate V3.0 (Con Protección de Back-EMF Automática).
 * @author Alejandro Moyano Crespillo (AleSMC)
 */

#include "include/strategy.h"
#include "include/RobotConfig.h"
#include "include/motor.h"
#include "include/sensors.h"
#include "include/timer.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

// --- VARIABLES DE ESTADO (Privadas) ---
static uint32_t g_attackStartTime = 0;
static bool g_isAttacking = false;

// Memoria de la última velocidad aplicada (para detectar cambios de sentido)
static int8_t g_lastSpeedLeft = 0;
static int8_t g_lastSpeedRight = 0;

// --- FUNCIÓN HELPER DE SEGURIDAD (MIDDLEWARE) ---
/**
 * @brief Aplica velocidad a los motores gestionando automáticamente el tiempo muerto.
 * @details Si detecta una inversión de polaridad (ej: de +100 a -100), 
 * fuerza una parada de 50ms para evitar picos inductivos.
 */
static void SetMotionSafe(int8_t new_left, int8_t new_right) {
    // Detectar Inversión de Giro (Zero Crossing)
    // Matemáticamente: Si (Actual * Nuevo) es negativo, tienen signos opuestos.
    // Usamos cast a (int) para evitar desbordamientos en la multiplicación.
    bool danger_left  = ((int)g_lastSpeedLeft * (int)new_left) < 0;
    bool danger_right = ((int)g_lastSpeedRight * (int)new_right) < 0;

    // Si alguno de los dos motores va a sufrir un "golpe de corriente"...
    if (danger_left || danger_right) {
        Motor_Stop();       // 1. Cortar energía
        Timer_WaitMillis(50); // 2. Esperar disipación de campo magnético (Deadtime)
    }

    // 3. Aplicar nueva velocidad
    Motor_SetSpeed(new_left, new_right);

    // 4. Actualizar memoria
    g_lastSpeedLeft = new_left;
    g_lastSpeedRight = new_right;
}

// --- API PÚBLICA ---

void Strategy_Init(void) {
    int i;
    // Secuencia de Arranque (5 Segundos)
    for (i = 0; i < 5; i++) {
        LED_RED_ON();
        Timer_WaitMillis(500);
        LED_RED_OFF();
        Timer_WaitMillis(500);
    }
    LED_RED_ON();
}

void Strategy_Run(void) {
    // 1. Leer Sensores
    Sensors_Update();
    SensorData_t data = Sensors_GetData();

    // --- PRIORIDAD 0: SUPERVIVENCIA (Línea) ---
    if (data.isLineDetected) {
        g_isAttacking = false;
        
        // Evasión Segura (La función SetMotionSafe se encarga del delay si hace falta)
        // 1. Atrás
        SetMotionSafe(-100, -100); 
        Timer_WaitMillis(250); // Tiempo de la maniobra
        
        // 2. Giro
        SetMotionSafe(100, -100);  
        Timer_WaitMillis(200); // Tiempo de la maniobra
        
        return; 
    }

    // --- PRIORIDAD 1: COMBATE (Bumpers) ---
    else if (data.isBumperLeft || data.isBumperRight) {
        g_isAttacking = true;
        g_attackStartTime = Millis();

        if (data.isBumperLeft && data.isBumperRight) {
            SetMotionSafe(100, -100); // Empuje Frontal
        }
        else if (data.isBumperLeft) {
            SetMotionSafe(20, -100);  // Giro contraataque IZQ
        }
        else {
            SetMotionSafe(100, -20);  // Giro contraataque DER
        }
    }

    // --- PRIORIDAD 2: ATAQUE (Sharp) ---
    else if (Sensors_IsEnemyClose()) {
        g_isAttacking = true;
        g_attackStartTime = Millis(); 
        SetMotionSafe(100, -100);     // Carga Frontal
    }
    
    // --- INERCIA (Blind Charge) ---
    else if (g_isAttacking && (Millis() - g_attackStartTime < 200)) {
        SetMotionSafe(100, -100);     // Mantener Carga
    } 
    
    // --- PRIORIDAD 3: BÚSQUEDA ---
    else {
        g_isAttacking = false;
        SetMotionSafe(60, 60);        // Giro Búsqueda
    }
}