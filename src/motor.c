/**
 * @file motor.c
 * @brief Implementación del driver de motores.
 * @details Manejo de bajo nivel de los periféricos PWM1 y GPIO Port F
 * para el control de servos futaba modificados.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#include "include/motor.h"
#include "include/RobotConfig.h" // Dependencia crítica: Configuración HAL

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

// --- FUNCIONES PRIVADAS (STATIC) ---

/**
 * @brief Mapea un porcentaje de velocidad a ticks de PWM.
 * @param speed Porcentaje de velocidad (-100 a 100).
 * @return uint32_t Ticks de ancho de pulso para el registro PWM.
 */
static uint32_t _map_speed(int8_t speed) {
    // 1. Protección de límites (Clamping)
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // 2. Retorno rápido si es parada
    if (speed == 0) return PWM_STOP_TICKS;

    int32_t ticks;
    
    // 3. Interpolación Lineal basada en las constantes de RobotConfig.h
    if (speed > 0) {
        // Sentido 1: Interpolamos entre STOP y MAX_CCW
        uint32_t range = PWM_DELTA_MAX_CCW - PWM_STOP_TICKS;
        ticks = PWM_STOP_TICKS + ((speed * range) / 100);
    } else {
        // Sentido 2: Interpolamos entre STOP y MAX_CW
        // Nota: speed es negativo, sumamos para restar del centro
        uint32_t range = PWM_STOP_TICKS - PWM_DELTA_MAX_CW;
        ticks = PWM_STOP_TICKS + ((speed * range) / 100);
    }

    return (uint32_t)ticks;
}

// --- IMPLEMENTACIÓN API PÚBLICA ---

void Motor_Init(void) {

    // 1. Configurar Divisor PWM (Requisito Hardware: /64)
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // 2. Habilitar Periféricos (Definidos en HAL)
    SysCtlPeripheralEnable(MOTORS_PWM_PERIPH);
    SysCtlPeripheralEnable(MOTORS_GPIO_PERIPH);

    // Espera activa segura (Wait for ready)
    while(!SysCtlPeripheralReady(MOTORS_PWM_PERIPH));

    // 3. Configurar Pines GPIO como PWM
    GPIOPinTypePWM(MOTORS_GPIO_BASE, MOTOR_LEFT_PIN | MOTOR_RIGHT_PIN);
    
    // Asignación de señales a pines (Pin Muxing)
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    // 4. Configurar Generador PWM 3 (Controla PWM6 y PWM7)
    PWMGenConfigure(MOTORS_PWM_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(MOTORS_PWM_BASE, PWM_GEN_3, PWM_PERIOD_TICKS);

    // 5. Estado Inicial: Motores Parados
    PWMPulseWidthSet(MOTORS_PWM_BASE, MOTOR_LEFT_PWM_OUT, PWM_STOP_TICKS);
    PWMPulseWidthSet(MOTORS_PWM_BASE, MOTOR_RIGHT_PWM_OUT, PWM_STOP_TICKS);

    // 6. Habilitar Salidas y Generador
    PWMOutputState(MOTORS_PWM_BASE, MOTOR_LEFT_PWM_BIT | MOTOR_RIGHT_PWM_BIT, true);
    PWMGenEnable(MOTORS_PWM_BASE, PWM_GEN_3);
}

void Motor_SetSpeed(int8_t speed_izq, int8_t speed_der) {
    // Cálculo de ticks
    uint32_t ticks_izq = _map_speed(speed_izq);
    uint32_t ticks_der = _map_speed(speed_der);

    // Aplicación al Hardware
    PWMPulseWidthSet(MOTORS_PWM_BASE, MOTOR_LEFT_PWM_OUT, ticks_izq);
    PWMPulseWidthSet(MOTORS_PWM_BASE, MOTOR_RIGHT_PWM_OUT, ticks_der);
}

void Motor_Stop(void) {
    PWMPulseWidthSet(MOTORS_PWM_BASE, MOTOR_LEFT_PWM_OUT, PWM_STOP_TICKS);
    PWMPulseWidthSet(MOTORS_PWM_BASE, MOTOR_RIGHT_PWM_OUT, PWM_STOP_TICKS);
}