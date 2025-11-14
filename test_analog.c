#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "drivers/rgb.h"
#include <stdint.h>
#include <stdbool.h>
// Librerias que se incluyen tipicamente para configuracion de perifericos y pinout
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
// Incluir librerias de perif�rico y otros que se vaya a usar para control PWM y gesti�n
// de botones  (TODO)

#include "driverlib/gpio.h" // Libreria GPIO
#include "driverlib/pwm.h" // Libreria PWM
#include "drivers/buttons.h" // Liberia botones TIVA Launchpad
#include "driverlib/interrupt.h" // Libreria Interrupciones
#include "inc/hw_ints.h" // Definiciones de interrupciones
#include <stdint.h>

// ===================== Configuration =====================
#define ADC0_THRESHOLD 1390 //1.12V rueda derecha

#define ADC1_THRESHOLD 1985  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6
#define ADC_HYSTERESIS       20  // adjust based on noise level

#define NUM_STEPS_PER_TURN_RD 18  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6
#define NUM_STEPS_PER_TURN_RI 18  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6

// ===================== Global Variables =====================
volatile uint32_t adc0Value;
volatile uint32_t adc1Value;

// Rising and falling edge flags
volatile bool adc0RisingEdge  = false;
volatile bool adc0FallingEdge = false;
volatile bool adc1RisingEdge  = false;
volatile bool adc1FallingEdge = false;

// Previous values for edge detection
volatile uint32_t adc0Prev = 0;
volatile uint32_t adc1Prev = 0;

volatile uint32_t counterRD = 0;
volatile uint32_t counterRI = 0;
volatile uint32_t counterWholeTurnsRD = 0;
volatile uint32_t counterWholeTurnsRI = 0;


// Previous values for edge detection


#define ADC_FILTER_SAMPLES 10

// Filter buffers and sums
volatile uint32_t adc0Buffer[ADC_FILTER_SAMPLES] = {0};
volatile uint32_t adc1Buffer[ADC_FILTER_SAMPLES] = {0};
volatile uint32_t adc0Sum = 0, adc1Sum = 0;
volatile uint8_t adc0Index = 0, adc1Index = 0;

// ===================== Function Prototypes =====================
void ADC0Seq3_Handler(void);
void ADC0Seq2_Handler(void);



//**********************MOTORES

#define PERIOD_PWM 12499 // (625KHz/50 Hz) -1   // TODO: Ciclos de reloj para conseguir una se�al peri�dica de 50Hz (seg�n reloj de perif�rico usado)
                            // PERIOD_PWM se obtendr�a como (pwm_clk / 50) - 1);  donde 50 es la frecuencia a obtener; y pwm_clk es la frecuencia de reloj del modulo PWM/Timer
                            // (se determina con SysCtlClockSet y  SysCtlPWMClockSet en el generador PWM; y SysCtlClockSet y las funciones de preescalado en los timers)
#define COUNT_1MS 624   // TODO: Ciclos en el ciclo de trabajo para amplitud de pulso de 1ms (max velocidad en un sentido)
//#define STOPCOUNT_RD 849 // (625KHz *1.52ms) -1 = 949 // TODO: Ciclos en el ciclo de trabajo  para amplitud de pulso de parada (1.52ms)

#define COUNT_2MS 1249   // TODO: Ciclos en el ciclo de trabajo  para amplitud de pulso de 2ms (max velocidad en el otro sentido)

// Los valores anteriores COUNT_XMS se obtendr�an como (pwm_clk * valor_en_ms_del_pulso/1000);
// Todos los valores anteriores tienen que ser de 16BITS m�ximo

#define NUM_STEPS 50    // Pasos/pulsaciones de boton necesarios para variar entre el pulso de 2ms al de 1ms (elegir a gusto del usuario)
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud del ciclo de trabajo tras pulsacion del boton

#define PWM_BASE_FREQ 50 // Esta es la frecuencia de la se�al PWM que queremos generar; en este caso concreto son 50Hz

uint32_t ui32DutyCycle1; //Variable que alamcenaran el ciclo de trabajo(entre 1 y 2 ms)
uint32_t ui32DutyCycle2; //Variable que alamcenaran el ciclo de trabajo(entre 1 y 2 ms)


volatile uint32_t STOPCOUNT_RD =945;
volatile uint32_t STOPCOUNT_RI =965;


//**********CONTROL DEL ROBOT


#include <stdint.h>
#include <stdbool.h>
#include <math.h> // Necesario para M_PI

// --- Includes de TivaWare (necesarios) ---
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
// ... (y cualquier otro driverlib que estés usando) ...


// --- Parámetros Físicos del Robot ---
#define R_CM 3.0f   // Radio de la rueda en cm
#define L_CM 9.3f   // Distancia entre ruedas en cm

// --- Constantes Calculadas ---
// Circunferencia = 2 * PI * R
#define WHEEL_CIRCUMFERENCE_CM (2.0f * M_PI * R_CM)
// Pulsos por cm = Pulsos por vuelta / Circunferencia
#define PULSES_PER_CM (NUM_STEPS_PER_TURN_RD / WHEEL_CIRCUMFERENCE_CM)
// Pulsos por grado de giro del robot:
// Derivación:
//   Arco (cm) = (Angulo_Robot_rad * L_CM) / 2.0f
//   Angulo_Robot_rad = Grados_Robot * (M_PI / 180.0f)
//   Arco (cm) = (Grados_Robot * (M_PI / 180.0f) * L_CM) / 2.0f
//   Pulsos_Arco = Arco (cm) * PULSES_PER_CM
//   Pulsos_Arco = (Grados_Robot * M_PI / 180.0 * L_CM / 2.0) * (NUM_STEPS_PER_TURN_RD / (2.0 * M_PI * R_CM))
//   Simplificando (M_PI se cancela):
//   Pulsos_Arco = Grados_Robot * (L_CM * NUM_STEPS_PER_TURN_RD) / (180.0 * 2.0 * 2.0 * R_CM)
#define PULSES_PER_DEGREE ( (L_CM * NUM_STEPS_PER_TURN_RD) / (720.0f * R_CM) ) // Aprox 0.0688 pulsos/grado

// --- Control de Motores (¡¡IMPORTANTE: AJUSTAR ESTOS VALORES!!) ---
// Asume un período de PWM (LOAD) ya configurado.
// ¡ESTOS VALORES SON SÓLO EJEMPLOS!
#define PWM_LOAD 40000 // ¡¡REEMPLAZAR con el valor de carga de tu PWM!!

// Asume 1.5ms STOP, 2.0ms FWD, 1.0ms REV y un período de 20ms (50Hz)
// 1.5ms / 20ms = 7.5% -> Duty = 0.075 * LOAD
#define FWD_DUTY  (uint32_t)  849 // Ejemplo: 2.0ms (Avance)
#define REV_DUTY  (uint32_t)    1049 // Ejemplo: 1.0ms (Reversa)


// --- Máquina de Estados del Robot ---
typedef enum {
    ROBOT_IDLE,     // Inactivo, esperando comandos
    ROBOT_MOVING,   // Moviéndose recto
    ROBOT_TURNING   // Girando
} RobotState;

// --- Variables Globales de Estado ---
volatile RobotState g_robot_state = ROBOT_IDLE;

// Objetivos (targets) de pulsos *relativos* al inicio
volatile int32_t g_target_counts_RD = 0;
volatile int32_t g_target_counts_RI = 0;

// Contadores en el momento de *inicio* del movimiento
volatile uint32_t g_start_counts_RD = 0;
volatile uint32_t g_start_counts_RI = 0;

void Robot_Init(void);
void mover_robot(float c);
void girar_robot(float g);
void Robot_Update(void);


int main(void)
{
    // Set system clock to 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    // Configure PD0 and PD1 as analog inputs
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);

    // ---- ADC Sequencer 3 -> PD0 -> AIN7 ----
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    ADCIntRegister(ADC0_BASE, 3, ADC0Seq3_Handler);
    ADCIntEnable(ADC0_BASE, 3);

    // ---- ADC Sequencer 2 -> PD1 -> AIN6 ----
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntRegister(ADC0_BASE, 2, ADC0Seq2_Handler);
    ADCIntEnable(ADC0_BASE, 2);


    ui32DutyCycle1 = STOPCOUNT_RD; // Inicializo el ciclo de trabajo al estado de reposo del servo.
      ui32DutyCycle2 = STOPCOUNT_RI;

      SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz;     // Elegir reloj adecuado para los valores de ciclos sean de tama�o soportable (cantidades menores de 16bits). Max frecuencia: 80MHz

    // Configura pulsadores placa TIVA (int. por flanco de bajada)
      ButtonsInit();
          GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
          GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
          IntEnable(INT_GPIOF);

    // Configuracion  ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1520us para salida por PF2, y COUNT_STOP para salida por PF3(o puedes poner otro valor si quieres que se mueva)
      // Opcion 1: Usar un Timer en modo PWM (ojo! Los timers PWM solo soportan cuentas
      //  de 16 bits, a menos que us�is un prescaler/timer extension)
      // Opcion 2: Usar un m�dulo PWM(no dado en Sist. Empotrados pero mas sencillo)
      //  SysCtlPWMClockSet(???);


      SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

      SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // Habilita puerto salida para se�al PWM (ver en documentacion que pin se corresponde a cada m�dulo PWM)

      // Rueda 1
      GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
      GPIOPinConfigure(GPIO_PF2_M1PWM6);

      // Rueda 2
      GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
      GPIOPinConfigure(GPIO_PF3_M1PWM7);

         // pwm_clk = SysCtlClockGet() / 64;  // 64, ya que es el factor de divisi�n del reloj de PWM respecto al reloj del sistema(SYSCTL_PWMDIV_64)
                                               // Si hubiesemos escogido otro SYSCTL_PWMDIV_XX, dividir�amos entre XX en vez de 64
             //val_load = (pwm_clk /PWM_BASE_FREQ  ) - 1; //IGUAL QUE PERIOD_PWM   //Valor del contador para la freq. pwm. En este caso --> 625Khz/50 = 6250-1: este valor NUNCA puede ser mayor que 0xFFFF!!
                                                         // Si te sale mayor, deberias usar un factor de divisi�n mayor (SYSCTL_PWMDIV_XX), o un reloj del sistema mas lento (SysCtlClockSet)

             PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // M�dulo PWM contara hacia abajo
             PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);    // Carga la cuenta que establece la frecuencia de la se�al PWM

//             PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);  // Establece el ciclo de trabajo R1
//             PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2);  // Establece el ciclo de trabajo R2

             PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la se�al R1
             PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la se�al R2

             PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM






    // Enable global interrupts
    IntMasterEnable();


    Robot_Init(); // Inicializa el estado del robot



    while(1)
    {

        Robot_Update();

        // Trigger ADC conversions
        ADCProcessorTrigger(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 2);

        static uint8_t checkEdgeRD=0;
            static uint8_t checkEdgeRI=0;
            static uint8_t flagLedRI=0;
            static uint8_t flagLedRD=0;

        // Optional small delay (~10ms)
        // SysCtlDelay(SysCtlClockGet()/100);

        // ================== ADC0 (Right Wheel) ==================
        if(adc0FallingEdge && checkEdgeRD == 1)
        {
            adc0FallingEdge = false;  // Clear after using
            checkEdgeRD = 0;           // Reset pending rising edge flag
            counterRD++;
            counterWholeTurnsRD = counterRD / NUM_STEPS_PER_TURN_RD;

            if(counterRD % NUM_STEPS_PER_TURN_RD == 0)
            {
                // Toggle green LED
                static uint8_t flagLedRD = 0;
                if(flagLedRD == 0)
                {

                    flagLedRD = 1;
                }
                else
                {

                    flagLedRD = 0;
                }
            }
        }

        if(adc0RisingEdge)
        {
            adc0RisingEdge = false; // Clear after processing
            checkEdgeRD = 1;        // Mark that a rising edge occurred
        }

        // ================== ADC1 (Left Wheel) ==================
        if(adc1FallingEdge && checkEdgeRI == 1)
        {
            adc1FallingEdge = false;  // Clear after using
            checkEdgeRI = 0;           // Reset pending rising edge flag
            counterRI++;
            counterWholeTurnsRI = counterRI / NUM_STEPS_PER_TURN_RI;

            if(counterRI % NUM_STEPS_PER_TURN_RI == 0)
            {
                // Toggle yellow LED
                static uint8_t flagLedRI = 0;
                if(flagLedRI == 0)
                {

                    flagLedRI = 1;
                }
                else
                {

                    flagLedRI = 0;
                }
            }
        }

        if(adc1RisingEdge)
        {
            adc1RisingEdge = false; // Clear after processing
            checkEdgeRI = 1;        // Mark that a rising edge occurred
        }
    }




}

// ===================== ADC Handlers =====================






void ADC0Seq3_Handler(void)
{
    uint32_t temp;
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &temp);

    // --- Simple moving average filter ---
    adc0Sum -= adc0Buffer[adc0Index];
    adc0Buffer[adc0Index] = temp;
    adc0Sum += adc0Buffer[adc0Index];
    adc0Index = (adc0Index + 1) % ADC_FILTER_SAMPLES;

    uint32_t filteredValue = adc0Sum / ADC_FILTER_SAMPLES;
    adc0Value = filteredValue;

    // Hysteresis thresholds
    uint32_t high = ADC0_THRESHOLD + ADC_HYSTERESIS;
    uint32_t low  = ADC0_THRESHOLD - ADC_HYSTERESIS;

    // Rising-edge detection with hysteresis
    if(adc0Prev <= high && filteredValue > high)

        adc0RisingEdge = true;

    // Falling-edge detection with hysteresis
    if(adc0Prev >= low && filteredValue < low)
        adc0FallingEdge = true;

    adc0Prev = filteredValue;
}

void ADC0Seq2_Handler(void)
{
    uint32_t temp;
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, &temp);

    // --- Simple moving average filter ---
    adc1Sum -= adc1Buffer[adc1Index];
    adc1Buffer[adc1Index] = temp;
    adc1Sum += adc1Buffer[adc1Index];
    adc1Index = (adc1Index + 1) % ADC_FILTER_SAMPLES;

    uint32_t filteredValue = adc1Sum / ADC_FILTER_SAMPLES;
    adc1Value = filteredValue;

    // Hysteresis thresholds
    uint32_t high = ADC1_THRESHOLD + ADC_HYSTERESIS;
    uint32_t low  = ADC1_THRESHOLD - ADC_HYSTERESIS;

    // Rising-edge detection with hysteresis
    if(adc1Prev <= high && filteredValue > high)
        adc1RisingEdge = true;

    // Falling-edge detection with hysteresis

    if(adc1Prev >= low && filteredValue < low)
        adc1FallingEdge = true;

    adc1Prev = filteredValue;
}



// Rutinas de interrupci�n de pulsadores
// Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a  COUNT_1MS
// Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a COUNT_2MS

void GPIOFIntHandler(void)
{
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);
    // Boton Izquierdo: reduce ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MINCOUNT
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){
        mover_robot(WHEEL_CIRCUMFERENCE_CM);

    }
    // Boton Derecho: aumenta ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MAXCOUNT
    if(((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)){ //Se ha pulsado boton derecho
        girar_robot(360);
    }
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}

void Timer0A_Handler(void) {
  //Añadida para que no de fallo al linkear
}



/**
 * @brief Función de ayuda para establecer la velocidad de ambos motores.
 * Asume que RI (Izquierda) es OUT_7 y RD (Derecha) es OUT_6.
 * @param duty_RI: Valor de "ancho de pulso" para Rueda Izquierda (PWM_OUT_7)
 * @param duty_RD: Valor de "ancho de pulso" para Rueda Derecha (PWM_OUT_6)
 */
void Set_Motor_Speeds(uint32_t duty_RI, uint32_t duty_RD) {
    // Rueda Derecha (RD) -> ui32DutyCycle1 -> PWM_OUT_6
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, duty_RD);
    // Rueda Izquierda (RI) -> ui32DutyCycle2 -> PWM_OUT_7
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, duty_RI);
}

/**
 * @brief Inicializa el robot. Debe llamarse una vez en el setup.
 */
void Robot_Init(void) {
    g_robot_state = ROBOT_IDLE;
    Set_Motor_Speeds(STOPCOUNT_RI, STOPCOUNT_RD); // Asegurarse de que los motores están parados
}

/**
 * @brief Inicia un movimiento recto del robot (No bloqueante).
 * @param c: Distancia en centímetros. (+) adelante, (-) atrás.
 */
void mover_robot(float c) {
    // No aceptar nuevos comandos si ya está ocupado
    if (g_robot_state != ROBOT_IDLE) {
        return;
    }

    // Calcular pulsos objetivo
    int32_t target_pulses = (int32_t)(c * PULSES_PER_CM);

    // Guardar estado inicial y objetivos
    // Usamos 'volatile' para asegurar que leemos el valor más actual
    g_start_counts_RD = counterRD;
    g_start_counts_RI = counterRI;

    g_target_counts_RD = target_pulses;
    g_target_counts_RI = target_pulses; // Ambas ruedas la misma distancia

    // Iniciar movimiento
    if (c > 0.0f) {
        g_robot_state = ROBOT_MOVING;
        Set_Motor_Speeds(FWD_DUTY, REV_DUTY);
    } else if (c < 0.0f) {
        g_robot_state = ROBOT_MOVING;
        Set_Motor_Speeds(REV_DUTY, FWD_DUTY);
    } else {
        // c == 0, no hacer nada
        g_robot_state = ROBOT_IDLE;
    }
}

/**
 * @brief Inicia un giro del robot sobre su propio eje (No bloqueante).
 * @param g: Ángulo en grados. (+) giro antihorario (izquierda),
 * (-) giro horario (derecha).
 */
void girar_robot(float g) {
    // No aceptar nuevos comandos si ya está ocupado
    if (g_robot_state != ROBOT_IDLE) {
        return;
    }

    // Calcular pulsos objetivo para el arco de giro
    int32_t target_arc_pulses = (int32_t)(g * PULSES_PER_DEGREE);

    // Guardar estado inicial
    g_start_counts_RD = counterRD;
    g_start_counts_RI = counterRI;

    if (g > 0.0f) {
        // Giro antihorario (izquierda)
        // Rueda derecha (RD) avanza, Rueda izquierda (RI) retrocede
        g_target_counts_RD = target_arc_pulses;
        g_target_counts_RI = target_arc_pulses;
        Set_Motor_Speeds(REV_DUTY, REV_DUTY); // RI(REV), RD(FWD)
        g_robot_state = ROBOT_TURNING;

    } else if (g < 0.0f) {
        // Giro horario (derecha)
        // Rueda derecha (RD) retrocede, Rueda izquierda (RI) avanza
        // target_arc_pulses ya será negativo
        g_target_counts_RD = target_arc_pulses; // negativo
        g_target_counts_RI = target_arc_pulses; // positivo
        Set_Motor_Speeds(FWD_DUTY, FWD_DUTY); // RI(FWD), RD(REV)
        g_robot_state = ROBOT_TURNING;

    } else {
        // g == 0, no hacer nada
        g_robot_state = ROBOT_IDLE;
    }
}

/**
 * @brief *** ¡¡FUNCIÓN CRÍTICA!! ***
 * Debe llamarse repetidamente en el bucle principal (main loop)
 * o desde una interrupción de temporizador (ej. SysTick).
 * Comprueba si el robot ha alcanzado su objetivo y lo detiene.
 */
void Robot_Update(void) {
    // Si no estamos haciendo nada, salir rápido.
    if (g_robot_state == ROBOT_IDLE) {
        return;
    }

    // Calcula los pulsos *transcurridos* desde el inicio del movimiento.
    // Esta resta y casting (int32_t) maneja correctamente el
    // desbordamiento (wraparound) de los contadores uint32_t,
    // ¡SIEMPRE Y CUANDO la ISR decremente en reversa!
    int32_t elapsed_RD = (int32_t)(counterRD - g_start_counts_RD);
    int32_t elapsed_RI = (int32_t)(counterRI - g_start_counts_RI);

    bool done_RD = false;
    bool done_RI = false;

    // --- Chequear Rueda Derecha (RD) ---
    if (g_target_counts_RD >= 0) { // Objetivo positivo (avance)
        if (elapsed_RD >= g_target_counts_RD) done_RD = true;
    } else { // Objetivo negativo (reversa)
        if (elapsed_RD <= g_target_counts_RD) done_RD = true;
    }

    // --- Chequear Rueda Izquierda (RI) ---
    if (g_target_counts_RI >= 0) { // Objetivo positivo (avance)
        if (elapsed_RI >= g_target_counts_RI) done_RI = true;
    } else { // Objetivo negativo (reversa)
        if (elapsed_RI <= g_target_counts_RI) done_RI = true;
    }

    // --- Comprobar si AMBOS han terminado ---
    if (done_RD && done_RI) {
        Set_Motor_Speeds(STOPCOUNT_RI, STOPCOUNT_RD); // ¡Parar!
        g_robot_state = ROBOT_IDLE;             // Volver a estado inactivo
    }
}
