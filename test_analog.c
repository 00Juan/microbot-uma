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


// ===================== Configuration =====================
#define ADC0_THRESHOLD 1390 //1.12V rueda derecha

#define ADC1_THRESHOLD 1985  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6
#define ADC_HYSTERESIS       20  // adjust based on noise level

#define NUM_STEPS_PER_TURN_RD 16  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6
#define NUM_STEPS_PER_TURN_RI 16  // Threshold for ADC1 (PD1/AIN6) //rueda izquierda 1.6

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

             PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);  // Establece el ciclo de trabajo R1
             PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2);  // Establece el ciclo de trabajo R2

             PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la se�al R1
             PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita la salida de la se�al R2

             PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM






    // Enable global interrupts
    IntMasterEnable();





    while(1)
    {

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
        if(ui32DutyCycle1 > COUNT_1MS)
        {
            ui32DutyCycle1 -= CYCLE_INCREMENTS;
            ui32DutyCycle2 -= CYCLE_INCREMENTS;
            if(ui32DutyCycle1 < COUNT_1MS)
            {
                ui32DutyCycle1 = COUNT_1MS;
                ui32DutyCycle2 = COUNT_1MS;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2);
        }
    }
    // Boton Derecho: aumenta ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF4, hasta llegar a MAXCOUNT
    if(((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)){ //Se ha pulsado boton derecho
        if(ui32DutyCycle1 < COUNT_2MS) //El ciclo de trabajo aun no esta en el maximo
        {
            ui32DutyCycle1 += CYCLE_INCREMENTS;
            ui32DutyCycle2 += CYCLE_INCREMENTS;
            if(ui32DutyCycle1 > COUNT_2MS) //Si el incremento hace que el ciclo supere el max le asignamos el max.
            {
                ui32DutyCycle1 = COUNT_2MS;
                ui32DutyCycle2 = COUNT_2MS;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle1); //Imponemos el nuevo ciclo de trabajo.
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle2); //Imponemos el nuevo ciclo de trabajo.
        }
    }
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}

void Timer0A_Handler(void) {
  //Añadida para que no de fallo al linkear
}

