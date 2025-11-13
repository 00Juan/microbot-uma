#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

// ================= Global Variables =================
volatile uint32_t adc0Value; // ADC channel 0 value
volatile uint32_t adc1Value; // ADC channel 1 value

int main(void)
{
    // Set system clock to 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable peripherals for ADC0 and GPIO Port D (PD0, PD1)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    // Configure PD0 and PD1 as analog inputs
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);

    // ---- Configure ADC Sequencers ----

    // Sequencer 3 -> PD0 -> AIN7
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    // Sequencer 2 -> PD1 -> AIN6
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);

    // ---- Main Loop: Read both channels ----
    while(1)
    {
        uint32_t temp;

        // Trigger Sequencer 3 (ADC0 AIN7)
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &temp);
        adc0Value = temp;

        // Trigger Sequencer 2 (ADC0 AIN6)
        ADCProcessorTrigger(ADC0_BASE, 2);
        while(!ADCIntStatus(ADC0_BASE, 2, false));
        ADCIntClear(ADC0_BASE, 2);
        ADCSequenceDataGet(ADC0_BASE, 2, &temp);
        adc1Value = temp;

        // Small delay (~10 ms)
        SysCtlDelay(SysCtlClockGet() / 100);
    }
}


void Timer0A_Handler()
{

}


void GPIOFIntHandler()
{

}
