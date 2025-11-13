#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"

// ===================== Configuration =====================
#define ADC0_THRESHOLD 2000  // Threshold for ADC0 (PD0/AIN7)
#define ADC1_THRESHOLD 2500  // Threshold for ADC1 (PD1/AIN6)

// ===================== Global Variables =====================
volatile uint32_t adc0Value;
volatile uint32_t adc1Value;

volatile bool adc0RisingEdge = false;
volatile bool adc1RisingEdge = false;

// Store previous values to detect rising edge
volatile uint32_t adc0Prev = 0;
volatile uint32_t adc1Prev = 0;

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

    // ---- Main loop ----
    while(1)
    {
        // Trigger ADC conversions
        ADCProcessorTrigger(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 2);

        // Small delay (~10 ms)
        SysCtlDelay(SysCtlClockGet()/100);

        // Handle rising-edge events
        if(adc0RisingEdge)
        {
            // Breakpoint here or handle ADC0 threshold rising edge
            adc0RisingEdge = false;
        }
        if(adc1RisingEdge)
        {
            // Breakpoint here or handle ADC1 threshold rising edge
            adc1RisingEdge = false;
        }
    }
}

// ===================== ADC Handlers =====================

// ADC0 Sequencer 3 -> PD0 -> AIN7
void ADC0Seq3_Handler(void)
{
    uint32_t temp;
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &temp);
    adc0Value = temp;

    // Rising-edge detection
    if(adc0Prev <= ADC0_THRESHOLD && adc0Value > ADC0_THRESHOLD)
    {
        adc0RisingEdge = true;  // Trigger event
    }

    adc0Prev = adc0Value;
}

// ADC0 Sequencer 2 -> PD1 -> AIN6
void ADC0Seq2_Handler(void)
{
    uint32_t temp;
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, &temp);
    adc1Value = temp;

    // Rising-edge detection
    if(adc1Prev <= ADC1_THRESHOLD && adc1Value > ADC1_THRESHOLD)
    {
        adc1RisingEdge = true;  // Trigger event
    }

    adc1Prev = adc1Value;
}



void Timer0A_Handler()
{

}


void GPIOFIntHandler()
{

}
