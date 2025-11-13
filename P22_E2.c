#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "drivers/rgb.h"

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

    // Enable global interrupts
    IntMasterEnable();


    char cColor;
    uint32_t pui32Color[3];
    /* The parameters are not used. */


    //Inicializa los LEDs...
    RGBInit(1);

    SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH); // Los Timers de PWM  tienen que seguir funcionando
    SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);  // aunque el micro este dormido
    SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);   // Redundante porque son el mismo, pero bueno...




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
                    pui32Color[0] = 0xFFFF; // Red
                    pui32Color[1] = 0;
                    pui32Color[2] = 0;
                    RGBColorSet(pui32Color);
                    flagLedRD = 1;
                }
                else
                {
                    pui32Color[0] = 0;
                    pui32Color[1] = 0;
                    pui32Color[2] = 0;
                    RGBColorSet(pui32Color);
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
                    pui32Color[0] = 0;
                    pui32Color[1] = 0xFFFF; // Yellow
                    pui32Color[2] = 0;
                    RGBColorSet(pui32Color);
                    flagLedRI = 1;
                }
                else
                {
                    pui32Color[0] = 0;
                    pui32Color[1] = 0;
                    pui32Color[2] = 0;
                    RGBColorSet(pui32Color);
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



void Timer0A_Handler()
{

}


void GPIOFIntHandler()
{

}
