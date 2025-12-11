/**
 * @file sensors.c
 * @brief Implementación del driver de sensores (ADC + GPIO).
 * @details Uso del ADC0 Sequence 1 para muestreo sincronizado.
 * * @author Alejandro Moyano Crespillo (AleSMC)
 * @version 1.0.0
 */

#include "include/sensors.h"
#include "include/RobotConfig.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

// --- VARIABLES PRIVADAS (Cache de estado) ---
static SensorData_t g_sensorState = {0};

// --- IMPLEMENTACIÓN ---

void Sensors_Init(void) {
    // 1. Habilitar Periféricos (Definidos en RobotConfig.h)
    SysCtlPeripheralEnable(SENSORS_ADC_PERIPH); // GPIO E (Analógicos)
    SysCtlPeripheralEnable(BUMPERS_PERIPH);     // GPIO A (Bumpers)
    SysCtlPeripheralEnable(ADC_PERIPH);         // ADC0 Module

    // Esperar a que estén listos
    while(!SysCtlPeripheralReady(ADC_PERIPH));

    // 2. Configurar Pines Analógicos (Sharp y Línea)
    GPIOPinTypeADC(SENSORS_ADC_BASE, SENSOR_SHARP_PIN | SENSOR_LINE_PIN);

    // 3. Configurar Pines Digitales (Bumpers) con Pull-Up interno
    // Bumpers suelen ser interruptores a tierra -> Pull-Up necesario.
    GPIOPinTypeGPIOInput(BUMPERS_BASE, BUMPER_LEFT_PIN | BUMPER_RIGHT_PIN);
    GPIOPadConfigSet(BUMPERS_BASE, BUMPER_LEFT_PIN | BUMPER_RIGHT_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Promediar 32 muestras automáticas. Reduce ruido drásticamente.
    // Opciones: 2, 4, 8, 16, 32, 64. 
    // 32 es el "Sweet Spot" para Sumo (Estable y Rápido).
    ADCHardwareOversampleConfigure(ADC_BASE, 32);

    // 4. CONFIGURACIÓN DEL ADC (La parte crítica)
    // Usaremos Secuenciador 1 (SS1) que permite hasta 4 muestras.
    ADCSequenceDisable(ADC_BASE, 1); // Buena práctica: Deshabilitar antes de configurar
    // Prioridad 0 (Máxima).
    ADCSequenceConfigure(ADC_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

    // Paso 0: Leer Sharp (CH1)
    ADCSequenceStepConfigure(ADC_BASE, 1, 0, ADC_CH_SHARP);
    
    // Paso 1: Leer Línea (CH2)
    // Este es el último paso, así que marcamos IE (Interrupt Enable) y END (End Sequence)
    ADCSequenceStepConfigure(ADC_BASE, 1, 1, ADC_CH_LINE | ADC_CTL_IE | ADC_CTL_END);

    // 5. Habilitar el Secuenciador
    ADCSequenceEnable(ADC_BASE, 1);
    
    // Limpiar bandera de interrupción por si acaso
    ADCIntClear(ADC_BASE, 1);
}

void Sensors_Update(void) {
    uint32_t adcValues[4]; // Buffer para recibir datos (SS1 soporta hasta 4)

    // 1. Disparar el ADC
    ADCProcessorTrigger(ADC_BASE, 1);

    // 2. Esperar conversión (Polling rápido)
    // Dado que el ADC es muy rápido (~1us), el bloqueo es despreciable aquí.
    while(!ADCIntStatus(ADC_BASE, 1, false)) {
        // Busy wait
    }
    
    // 3. Limpiar bandera
    ADCIntClear(ADC_BASE, 1);

    // 4. Leer datos del FIFO
    // Devuelve el número de muestras leídas.
    ADCSequenceDataGet(ADC_BASE, 1, adcValues);

    // 5. Actualizar Estado Interno (Cache)
    g_sensorState.rawSharp = adcValues[0]; // Paso 0 configurado arriba
    g_sensorState.rawLine  = adcValues[1]; // Paso 1 configurado arriba

    // 6. Leer Digitales (Bumpers)
    // Lógica negativa (0 = presionado) debido al Pull-Up
    // Usamos las MACROS definidas en RobotConfig.h si es posible, o lectura directa.
    // Aquí implementamos lectura directa para ser consistentes con sensors.c
    int32_t pinState = GPIOPinRead(BUMPERS_BASE, BUMPER_LEFT_PIN | BUMPER_RIGHT_PIN);
    
    g_sensorState.isBumperLeft  = ((pinState & BUMPER_LEFT_PIN) == 0);
    g_sensorState.isBumperRight = ((pinState & BUMPER_RIGHT_PIN) == 0);

    // 7. Procesar Lógica de Umbrales (Comparadores)
    
    // SHARP: Voltaje sube al acercarse. Si valor > Umbral -> Detectado.
    g_sensorState.isEnemyDetected = (g_sensorState.rawSharp > SHARP_THRESHOLD_TICKS);

    // LINEA (CNY70):
    // Negro (Suelo) = Poco reflejo = Voltaje Alto (Pull-up dominante) -> Valor ADC Alto
    // Blanco (Borde) = Mucho reflejo = Voltaje Bajo (Fototransistor conduce) -> Valor ADC Bajo
    // OJO: Depende del circuito exacto.
    // Asumiremos lógica estándar: Blanco = ADC BAJO.
    // Si tu lógica es inversa (Blanco = Alto), cambia el operador aquí.
    // Según RobotConfig: "Umbral seguro ~1860".
    // Si ADC < 1860 estamos en blanco (PELIGRO).
    // Si ADC > 1860 estamos en negro (SAFE).
    // isLineDetected = PELIGRO.
    g_sensorState.isLineDetected = (g_sensorState.rawLine < LINE_THRESHOLD_TICKS);
}

SensorData_t Sensors_GetData(void) {
    return g_sensorState;
}

bool Sensors_IsEnemyClose(void) {
    return g_sensorState.isEnemyDetected;
}

bool Sensors_IsLineDetected(void) {
    return g_sensorState.isLineDetected;
}