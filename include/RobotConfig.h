/**
 * @file RobotConfig.h
 * @brief Configuración Hardware Abstraction Layer (HAL) para SumoBOT.
 * @details Define el mapeo de pines, constantes del sistema, calibración de servos
 * y umbrales de sensores para Tiva C TM4C123GH6PM.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* =================================================================================
 * 1. CONFIGURACIÓN DEL SISTEMA (CLOCK)
 * ================================================================================= */
/** * @brief Frecuencia del Reloj del Sistema.
 * Configurado en main.c con SYSCTL_SYSDIV_5 (200 MHz / 5 = 40 MHz).
 */
#define SYSTEM_CLOCK_HZ 40000000U

/* =================================================================================
 * 2. ACTUADORES: MOTORES (SERVOS TRUCADOS)
 * ================================================================================= */
/* Hardware Map: Motores */
#define MOTORS_GPIO_PERIPH SYSCTL_PERIPH_GPIOF
#define MOTORS_GPIO_BASE GPIO_PORTF_BASE
#define MOTORS_PWM_PERIPH SYSCTL_PERIPH_PWM1
#define MOTORS_PWM_BASE PWM1_BASE

/* Pines Específicos */
#define MOTOR_LEFT_PIN GPIO_PIN_2  // PF2 -> M1PWM6
#define MOTOR_RIGHT_PIN GPIO_PIN_3 // PF3 -> M1PWM7
#define MOTOR_LEFT_PWM_OUT PWM_OUT_6
#define MOTOR_LEFT_PWM_BIT PWM_OUT_6_BIT
#define MOTOR_RIGHT_PWM_OUT PWM_OUT_7
#define MOTOR_RIGHT_PWM_BIT PWM_OUT_7_BIT

/* Calibración PWM (50 Hz -> 20ms) - Basado en Clock 40MHz / 64 PWM Divisor */
/* Ticks = (40,000,000 / 64) / 50 = 12500 Ticks por periodo */
#define PWM_PERIOD_TICKS 12499U // (12500 - 1)
#define PWM_STOP_TICKS 949U     // 1.5ms (Punto neutro)

/* Delta de velocidad para movimientos máximos */
/* Nota: Usamos defines para evitar cálculos en tiempo de ejecución */
#define PWM_DELTA_MAX_CW 624U   // Delta para 1ms
#define PWM_DELTA_MAX_CCW 1249U // Delta para 2ms

/* =================================================================================
 * 3. SENSORES: VISIÓN Y DISTANCIA (ADC)
 * ================================================================================= */
/* * Puerto E es ideal para analógicos en Tiva C.
 * Asumimos PE2 para Sharp y PE1/PE3 para Línea según layouts estándar de expansión.
 */
#define SENSORS_ADC_PERIPH SYSCTL_PERIPH_GPIOE
#define SENSORS_ADC_BASE GPIO_PORTE_BASE
#define ADC_PERIPH SYSCTL_PERIPH_ADC0
#define ADC_BASE ADC0_BASE

/* Pines de Sensores */
#define SENSOR_SHARP_PIN GPIO_PIN_2 // PE2 (AIN1) - Distancia
#define SENSOR_LINE_PIN GPIO_PIN_1  // PE1 (AIN2) - Suelo/Línea

/* Canales ADC correspondientes */
#define ADC_CH_SHARP ADC_CTL_CH1
#define ADC_CH_LINE ADC_CTL_CH2

/* * UMBRALES DE SENSORES (Calibración)
 * ADC es 12-bit (0 - 4095). Vref = 3.3V.
 */

/* * Calibración Sharp (Basado en CaracterizacionSharp.csv):
 * 20 cm = 0.61 V
 * ADC = (0.61 / 3.3) * 4096 ~= 757
 * Definimos umbral de ataque en 760 (aprox 20cm).
 * Si Valor > UMBRAL -> Objeto cerca (Voltaje sube al acercarse).
 */
#define SHARP_THRESHOLD_TICKS 760U

/* * Calibración Línea (CNY70):
 * Blanco (Tatami) refleja luz -> Voltaje bajo (típicamente < 1V).
 * Negro (Borde) absorbe luz -> Voltaje alto (pull-up actúa).
 * Umbral seguro: ~1.5V -> ADC ~1860.
 */
#define LINE_THRESHOLD_TICKS 1860U

/* =================================================================================
 * 4. SENSORES: TACTO (DIGITAL)
 * ================================================================================= */
/* Puerto A suele usarse para Bumpers en placas de expansión por accesibilidad */
#define BUMPERS_PERIPH SYSCTL_PERIPH_GPIOA
#define BUMPERS_BASE GPIO_PORTA_BASE

#define BUMPER_LEFT_PIN GPIO_PIN_2  // PA2
#define BUMPER_RIGHT_PIN GPIO_PIN_3 // PA3

/* Macros de lectura eficiente (Evita llamadas a función costosas en bucles críticos) */
/* Devuelve true si el bumper está presionado (Lógica Negativa habitual: 0 = presionado) */
#define READ_BUMPER_LEFT() (GPIOPinRead(BUMPERS_BASE, BUMPER_LEFT_PIN) == 0)
#define READ_BUMPER_RIGHT() (GPIOPinRead(BUMPERS_BASE, BUMPER_RIGHT_PIN) == 0)

/* =================================================================================
 * 5. INTERFAZ DE USUARIO / DEBUG
 * ================================================================================= */
/* Led Rojo de la Tiva (PF1) para indicar estado (ej. Parada o Detección) */
#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_BASE GPIO_PORTF_BASE
#define LED_RED_PIN GPIO_PIN_1

#define LED_RED_ON() GPIOPinWrite(LED_BASE, LED_RED_PIN, LED_RED_PIN)
#define LED_RED_OFF() GPIOPinWrite(LED_BASE, LED_RED_PIN, 0)

/* =================================================================================
 * 6. CONFIGURACIÓN ODOMETRÍA (SINGLE ENCODER CNY70 - PC5)
 * ================================================================================= */
// Usamos PC5 (Interrupción Digital)
#define ENCODER_GPIO_PERIPH     SYSCTL_PERIPH_GPIOC
#define ENCODER_GPIO_BASE       GPIO_PORTC_BASE
#define ENCODER_PIN             GPIO_PIN_5
#define ENCODER_INT             INT_GPIOC

// --- FÍSICA DEL ROBOT (CALIBRAR) ---
// Radio de la rueda en cm
#define WHEEL_RADIUS_CM         2.0f   
// Distancia entre el centro de las dos ruedas (Ancho del robot)
#define WHEEL_BASE_CM           10.0f  

// Resolución del Disco Encoder (Franjas negras)
#define STRIPES_COUNT           8      
// Interrupción en ambos flancos (Blanco->Negro y Negro->Blanco) = Doble resolución
#define TICKS_PER_REV           (STRIPES_COUNT * 2) 

// Cálculos automáticos (Perímetro y cm por tick)
#define WHEEL_PERIMETER_CM      (2.0f * 3.141592f * WHEEL_RADIUS_CM)
#define CM_PER_TICK             (WHEEL_PERIMETER_CM / (float)TICKS_PER_REV)

#endif /* ROBOT_CONFIG_H */
