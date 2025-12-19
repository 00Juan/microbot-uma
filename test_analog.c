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

#define NUM_STEPS_PER_TURN_RD 7  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6
#define NUM_STEPS_PER_TURN_RI 7  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6

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


const uint32_t STOPCOUNT_RI =945;
const uint32_t STOPCOUNT_RD =965;




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
//#define FWD_DUTY  (uint32_t)  700 // Ejemplo: 2.0ms (Avance)
//#define REV_DUTY  (uint32_t)    1200 // Ejemplo: 1.0ms (Reversa)
#define INCREMENT 400

#define FWD_DUTY_RI STOPCOUNT_RI-INCREMENT
#define FWD_DUTY_RD STOPCOUNT_RD-INCREMENT

#define REV_DUTY_RI STOPCOUNT_RI+INCREMENT
#define REV_DUTY_RD STOPCOUNT_RD+INCREMENT


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


// *** NUEVO: Almacena el DutyCycle de "movimiento" para cada rueda ***
volatile uint32_t g_running_duty_RD = STOPCOUNT_RD;
volatile uint32_t g_running_duty_RI = STOPCOUNT_RI;

// Añadir esto junto a tus otras variables globales

typedef enum {
    SEQ_IDLE,       // No hay ninguna secuencia activa
    SEQ_SQUARE      // La secuencia "cuadrado" está activa
} SequenceState;

volatile SequenceState g_current_sequence = SEQ_IDLE;
volatile uint8_t g_sequence_step = 0; // Para contar los 8 pasos (4x mover, 4x girar)


// --- Global Variables & Prototypes ---
volatile uint32_t g_ui32PortE_Presses = 0; // Counter for Port E buttons
void PortE_IntHandler(void);               // Prototype for the new ISR



#define WHISKER_R             1
#define WHISKER_L            2

volatile int stsWhiskerR=0;
volatile int stsWhiskerL=0;
volatile uint32_t stsSharp=0;
void ADC0Seq1_Handler(void);



void Robot_Init(void);
void mover_robot(float c);
void girar_robot(float g);
void Robot_Update(void);
void iniciar_secuencia_cuadrado(void);
void Sequence_Update(void);


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

    // ---- NEW: Configure PD3 (AIN4) as analog input ----
        // We add PD3 to the configuration. No new Port Enable needed!
        GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
        GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);

        // ---- NEW: ADC Sequencer 1 -> PD3 -> AIN4 ----
            // We use Sequencer 1 because Seq 3 and 2 are taken.
            ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

            // Configure step 0 of Seq 1 to read Channel 4 (AIN4 = PD3)
            ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);

            ADCSequenceEnable(ADC0_BASE, 1);
            ADCIntClear(ADC0_BASE, 1);
            ADCIntRegister(ADC0_BASE, 1, ADC0Seq1_Handler); // Don't forget to create this function!
            ADCIntEnable(ADC0_BASE, 1);


    ui32DutyCycle1 = STOPCOUNT_RD; // Inicializo el ciclo de trabajo al estado de reposo del servo.
    ui32DutyCycle2 = STOPCOUNT_RI;

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Reloj del sistema a 40MHz;     // Elegir reloj adecuado para los valores de ciclos sean de tama�o soportable (cantidades menores de 16bits). Max frecuencia: 80MHz



    // --- 3. NEW: Configure Port E (PE0 & PE1) for Interrupts ---
        // We do this here so it uses the final 40MHz clock.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);       // Enable Port E
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Wait for ready

        // Configure PE0 and PE1 as Inputs
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_1);

        // Enable Internal Pull-Up Resistors (Since buttons connect to Ground)
        GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        // Register and Enable Port E Interrupts
//        GPIOIntRegister(GPIO_PORTE_BASE, PortE_IntHandler); // Register ISR
//        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_1, GPIO_FALLING_EDGE); // Trigger on Press
//        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_1); // Enable Pins



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

    int contResetWhisker=0;

    while(1)
    {

        ADCProcessorTrigger(ADC0_BASE, 1);
        Robot_Update();
        Sequence_Update();

        // 1. POLL THE WHISKERS (Digital Read)
                // We read the pins directly every time the loop runs.
                int32_t i32PinValues = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);

                // Check PE2 (Right Whisker) - Low (0) means Pressed
                if( (i32PinValues & GPIO_PIN_2) == 0 )
                {
                    stsWhiskerR = 1;
                }
                else
                {
                    stsWhiskerR = 0;
                }

                // Check PE1 (Left Whisker) - Low (0) means Pressed
                if( (i32PinValues & GPIO_PIN_1) == 0 )
                {
                    stsWhiskerL = 1;
                }
                else
                {
                    stsWhiskerL = 0;
                }


                static  int sharpLimit= 1000;


                if(adc1Value>2500 || stsWhiskerR==1 ||stsWhiskerL==1) //Linea blanca
                {
                    mover_robot(-3);
                    //girar_robot(180);
                }
                else if(adc1Value<2500 && (stsWhiskerR==1 ||stsWhiskerL==1))
                {
                    girar_robot(90);

                }

                else if(adc1Value<=2500 && ( stsSharp>sharpLimit && stsWhiskerR==0 && stsWhiskerL==0 )) //Sin linea blanca y detecta robot
                {
                    mover_robot(3);

                }
                else if ( adc1Value<=2500 && ( stsSharp< sharpLimit && stsWhiskerR==0 && stsWhiskerL==0))
                {
                    girar_robot(45);
                }




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


    }




}


/**
 * @brief Inicia la secuencia de "hacer un cuadrado".
 * La secuencia es (mover 10cm, girar 90º) x 4.
 * Es NO bloqueante.
 */
void iniciar_secuencia_cuadrado(void) {
    // No iniciar una nueva secuencia si el robot o otra secuencia ya están ocupados
    if (g_robot_state != ROBOT_IDLE || g_current_sequence != SEQ_IDLE) {
        return;
    }

    // Configurar la máquina de estados de secuencia
    g_current_sequence = SEQ_SQUARE;
    g_sequence_step = 0; // Estamos en el paso 0 (el primer movimiento)

    // Lanzar la *primera* acción de la secuencia
    mover_robot(50.0);
}


/**
 * @brief *** ¡FUNCIÓN CRÍTICA DE SECUENCIA! ***
 * Debe llamarse repetidamente en el bucle principal (main loop)
 * después de Robot_Update().
 * Comprueba si la acción actual terminó y lanza la siguiente.
 */
void Sequence_Update(void) {
    // Si no hay secuencia activa, no hacer nada.
    if (g_current_sequence == SEQ_IDLE) {
        return;
    }

    // Si el robot todavía está ocupado (moviéndose o girando),
    // no hacer nada y esperar.
    if (g_robot_state != ROBOT_IDLE) {
        return;
    }

    // --- Si llegamos aquí, significa que: ---
    // 1. Hay una secuencia activa (g_current_sequence != SEQ_IDLE)
    // 2. El robot acaba de terminar su última acción (g_robot_state == ROBOT_IDLE)
    //
    // ¡Es hora de lanzar el siguiente paso!

    g_sequence_step++; // Avanzamos al siguiente paso

    // Lógica de la secuencia "Cuadrado"
    if (g_current_sequence == SEQ_SQUARE) {

        // La secuencia tiene 8 pasos en total (0-7):
        // 0: move, 1: turn, 2: move, 3: turn, 4: move, 5: turn, 6: move, 7: turn

        if (g_sequence_step == 1 || g_sequence_step == 3 || g_sequence_step == 5 || g_sequence_step == 7) {
            // El robot acaba de TERMINAR un MOVIMIENTO (pasos 0, 2, 4, 6).
            // Ahora TOCA GIRAR.
            girar_robot(90.0); // Asumiendo +90º = giro a la izquierda

        } else if (g_sequence_step == 2 || g_sequence_step == 4 || g_sequence_step == 6) {
            // El robot acaba de TERMINAR un GIRO (pasos 1, 3, 5).
            // Ahora TOCA MOVERSE.
            mover_robot(50.0);

        } else if (g_sequence_step >= 8) {
            // El robot acaba de TERMINAR el último GIRO (paso 7).
            // La secuencia se ha completado.
            g_current_sequence = SEQ_IDLE;
            g_sequence_step = 0;
        }
    }
}




/**
 * @brief Función de ayuda para establecer la velocidad de ambos motores.
 * Asume que RI (Izquierda) es OUT_7 y RD (Derecha) es OUT_6.
 * @param duty_RI: Valor de "ancho de pulso" para Rueda Izquierda (PWM_OUT_7)
 * @param duty_RD: Valor de "ancho de pulso" para Rueda Derecha (PWM_OUT_6)
 */
void Set_Motor_Speeds(uint32_t duty_RI, uint32_t duty_RD) {
    // Rueda Derecha (RD) -> ui32DutyCycle1 -> PWM_OUT_6
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, duty_RI);
    // Rueda Izquierda (RI) -> ui32DutyCycle2 -> PWM_OUT_7
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, duty_RD);
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
/**
 * @brief Inicia un movimiento recto del robot (No bloqueante).
 * @param c: Distancia en centímetros. (+) adelante, (-) atrás.
 */
void mover_robot(float c) {
    // No aceptar nuevos comandos si ya está ocupado
    if (g_robot_state != ROBOT_IDLE) {
        return;
    }

    // Calcular pulsos objetivo (siempre positivo)
    uint32_t target_pulses = (uint32_t)(fabsf(c) * PULSES_PER_CM);

    // Si el objetivo es 0, no hacer nada
    if (target_pulses == 0) {
        g_robot_state = ROBOT_IDLE;
        return;
    }

    // Guardar estado inicial y objetivos
    g_start_counts_RD = counterRD;
    g_start_counts_RI = counterRI;

    g_target_counts_RD = target_pulses;
    g_target_counts_RI = target_pulses;

    // Iniciar movimiento y ALMACENAR la dirección
    if (c > 0.0f) {
        // Hacia adelante
        g_running_duty_RD = FWD_DUTY_RD;
        g_running_duty_RI = REV_DUTY_RI;
    } else {
        // Hacia atrás (c < 0.0f)
        g_running_duty_RD = REV_DUTY_RD;
        g_running_duty_RI = FWD_DUTY_RI;
    }

    // Establecer estado y aplicar velocidad inicial
    g_robot_state = ROBOT_MOVING;
    Set_Motor_Speeds(g_running_duty_RI, g_running_duty_RD);
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

    // Calcular pulsos objetivo (siempre positivo)
    uint32_t target_arc_pulses = (uint32_t)(fabsf(g) * PULSES_PER_DEGREE);

    // Si el objetivo es 0, no hacer nada
    if (target_arc_pulses == 0) {
        g_robot_state = ROBOT_IDLE;
        return;
    }

    // Guardar estado inicial y objetivos
    g_start_counts_RD = counterRD;
    g_start_counts_RI = counterRI;

    g_target_counts_RD = target_arc_pulses;
    g_target_counts_RI = target_arc_pulses;

    // Iniciar movimiento y ALMACENAR las direcciones de giro
    if (g > 0.0f) {
        // Giro antihorario (izquierda)
        // Rueda derecha (RD) avanza, Rueda izquierda (RI) retrocede
        g_running_duty_RD = FWD_DUTY_RD;
        g_running_duty_RI = FWD_DUTY_RI;
    } else {
        // Giro horario (derecha) (g < 0.0f)
        // Rueda derecha (RD) retrocede, Rueda izquierda (RI) avanza
        g_running_duty_RD = REV_DUTY_RD;
        g_running_duty_RI = REV_DUTY_RI;
    }

    // Establecer estado y aplicar velocidad inicial
    g_robot_state = ROBOT_TURNING;
    Set_Motor_Speeds(g_running_duty_RI, g_running_duty_RD);
}

/**
 * @brief *** ¡FUNCIÓN CRÍTICA CORREGIDA (versión 2)! ***
 * Comprueba si el robot ha alcanzado su objetivo y lo detiene.
 * ASUME: counterRD y counterRI solo incrementan.
 */
void Robot_Update(void) {
    // Si no estamos haciendo nada, salir rápido.
    if (g_robot_state == ROBOT_IDLE) {
        return;
    }

    // Calcula los pulsos *transcurridos* desde el inicio del movimiento.
    // Esta resta (uint32_t) maneja correctamente el desbordamiento (wraparound)
    // siempre que counterRD/RI solo incrementen.
    uint32_t elapsed_RD = counterRD - g_start_counts_RD;
    uint32_t elapsed_RI = counterRI - g_start_counts_RI;

    bool done_RD = false;
    bool done_RI = false;

    // --- Chequear Ruedas (Lógica simplificada) ---
    // Dado que 'elapsed' y 'target' son ambos uint32_t positivos,
    // la comprobación es una simple comparación "mayor o igual que".
    if (elapsed_RD >= g_target_counts_RD) {
        done_RD = true;
    }
    if (elapsed_RI >= g_target_counts_RI) {
        done_RI = true;
    }

    // --- Lógica de Control Independiente ---
    uint32_t next_duty_RD;
    uint32_t next_duty_RI;

    // --- Decidir Rueda Derecha (RD) ---
    if (done_RD) {
        // La rueda RD ha terminado, debe PARAR.
        next_duty_RD = STOPCOUNT_RD;
        next_duty_RI = STOPCOUNT_RI;
    } else {
        // La rueda RD no ha terminado, debe SEGUIR MOVIÉNDOSE.
        // ¿En qué dirección? En la que guardamos al inicio.
        next_duty_RD = g_running_duty_RD;
        next_duty_RI = g_running_duty_RI;
    }

//    // --- Decidir Rueda Izquierda (RI) ---
//    if (done_RI) {
//        // La rueda RI ha terminado, debe PARAR.
//        next_duty_RI = STOPCOUNT_RI;
//    } else {
//        // La rueda RI no ha terminado, debe SEGUIR MOVIÉNDOSE.
//        next_duty_RI = g_running_duty_RI;
//    }

    // 2. Aplicar las velocidades (una podría ser STOP y la otra no)
    Set_Motor_Speeds(next_duty_RI, next_duty_RD);

    // 3. Comprobar si AMBOS han terminado para liberar la máquina de estados
    if (done_RD) { //Poner done_RD && done_RI si están los dos encoders puestos
        // Ambas ruedas están paradas, el robot está 'IDLE'.
        g_robot_state = ROBOT_IDLE;

        // Limpiar las direcciones "running" por seguridad
        g_running_duty_RD = STOPCOUNT_RD;
        g_running_duty_RI = STOPCOUNT_RI;
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
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS2);
    // Boton Izquierdo: reduce ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MINCOUNT
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){
       iniciar_secuencia_cuadrado();
        //girar_robot(180);

    }
    // Boton Derecho: aumenta ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MAXCOUNT
    if(((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)){ //Se ha pulsado boton derecho

        mover_robot(WHEEL_CIRCUMFERENCE_CM);
    }





    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}

void Timer0A_Handler(void) {
    //Añadida para que no de fallo al linkear
}



// Interrupt Handler for Port E (PE0 and PE1)
void PortE_IntHandler(void)
{
//    uint32_t ui32Status;
//
//
//    // Get interrupt status. True means "return masked status"
//    ui32Status = GPIOIntStatus(GPIO_PORTE_BASE, true);
//
//    // Clear the interrupt flag immediately
//    GPIOIntClear(GPIO_PORTE_BASE, ui32Status);
//
//    // Check if PE0 was the cause
//    if(ui32Status & GPIO_PIN_2)
//    {
//        // TODO: Add code for Button connected to PE0
//        stsWhiskerR=1;
//    }
//
//    // Check if PE1 was the cause
//    if(ui32Status & GPIO_PIN_1)
//    {
//        // TODO: Add code for Button connected to PE1
//        stsWhiskerL=1;
//    }
}

// ISR for the new PD3 Analog Input
void ADC0Seq1_Handler(void)
{
    uint32_t ui32ADCValuePD3[1];

    // 1. Clear the interrupt flag so it doesn't loop forever
    ADCIntClear(ADC0_BASE, 1);

    // 2. Read the data from Sequencer 1
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADCValuePD3);

    stsSharp=ui32ADCValuePD3[0];


    // ui32ADCValuePD3[0] is your new value (0-4095)
    // Example: g_SensorValue = ui32ADCValuePD3[0];
}
