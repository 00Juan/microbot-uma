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




#define ADC_FILTER_SAMPLES 100

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




    // ---- Main loop ----
    while(1)
    {
        static uint8_t checkEdge=0;
        // Trigger ADC conversions
        ADCProcessorTrigger(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 2);

        // Small delay (~10 ms)
         //SysCtlDelay(SysCtlClockGet()/100);

        // Example: handle edge events
        if(adc0RisingEdge)
        {

            adc0RisingEdge = false;


            // Handle rising event for ADC0
        }
        if(adc0FallingEdge)
        {
            adc0FallingEdge = false;
//            pui32Color[0]=0;
//            pui32Color[1]=0xFFFF;
//            pui32Color[2]=0;
//            RGBColorSet(pui32Color);
            // Handle falling event for ADC0
        }
        if(adc1RisingEdge)
        {
            adc1RisingEdge = false;
//            pui32Color[0]=0xFFFF;
//            pui32Color[1]=0;
//            pui32Color[2]=0;
//            RGBColorSet(pui32Color);
            counterRI++;
        }
        if(adc1FallingEdge)
        {
            adc1FallingEdge = false;
//            pui32Color[0]=0;
//            pui32Color[1]=0xFFFF;
//            pui32Color[2]=0;
//            RGBColorSet(pui32Color);
        }

        if(adc0RisingEdge && checkEdge==0)
        {
            checkEdge=1;
        }
        else if(adc0FallingEdge && checkEdge==1)
        {
            static uint8_t flagLed=0;
            checkEdge=0;
            counterRD++;
            counterWholeTurnsRD=counterRD/NUM_STEPS_PER_TURN_RD;
            if(counterRD%NUM_STEPS_PER_TURN_RD==0)
            {
                if(flagLed==0)
                {
                    pui32Color[0]=0xFFFF;
                    pui32Color[1]=0;
                    pui32Color[2]=0;
                    RGBColorSet(pui32Color);
                    flagLed=1;
                }
                else
                {
                    pui32Color[0]=0;
                    pui32Color[1]=0xFFFF;
                    pui32Color[2]=0;
                    RGBColorSet(pui32Color);
                    flagLed=0;
                }
            }
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
    adc0Sum -= adc0Buffer[adc0Index];  // remove oldest value
    adc0Buffer[adc0Index] = temp;      // store new sample
    adc0Sum += adc0Buffer[adc0Index];  // add new value
    adc0Index = (adc0Index + 1) % ADC_FILTER_SAMPLES;

    // Filtered value
    uint32_t filteredValue = adc0Sum / ADC_FILTER_SAMPLES;
    adc0Value = filteredValue;

    // Rising-edge detection
    if(adc0Prev <= ADC0_THRESHOLD && filteredValue > ADC0_THRESHOLD)
        adc0RisingEdge = true;

    // Falling-edge detection
    if(adc0Prev > ADC0_THRESHOLD && filteredValue <= ADC0_THRESHOLD)
        adc0FallingEdge = true;

    adc0Prev = filteredValue;
}

// ADC0 Sequencer 2 -> PD1 -> AIN6
void ADC0Seq2_Handler(void)
{
    uint32_t temp;
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, &temp);

    // --- Simple moving average filter ---
    adc1Sum -= adc1Buffer[adc1Index];  // remove oldest value
    adc1Buffer[adc1Index] = temp;      // store new sample
    adc1Sum += adc1Buffer[adc1Index];  // add new value
    adc1Index = (adc1Index + 1) % ADC_FILTER_SAMPLES;

    // Filtered value
    uint32_t filteredValue = adc1Sum / ADC_FILTER_SAMPLES;
    adc1Value = filteredValue;

    // Rising-edge detection
    if(adc1Prev <= ADC1_THRESHOLD && filteredValue > ADC1_THRESHOLD)
        adc1RisingEdge = true;

    // Falling-edge detection
    if(adc1Prev > ADC1_THRESHOLD && filteredValue <= ADC1_THRESHOLD)
        adc1FallingEdge = true;

    adc1Prev = filteredValue;
}


void Timer0A_Handler()
{

}


void GPIOFIntHandler()
{

}
