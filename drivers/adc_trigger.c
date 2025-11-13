#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

// Static (private) variable to hold the callback function pointer
static ADCCallback g_ADCCallback = 0;

// Internal ADC Interrupt Handler
void ADC0SS0_Handler(void)
{
    uint32_t ui32CompStatus;
    uint32_t pui32Data[2]; // Sequencer 0 has 2 steps

    // --- FIX 1 (CRITICAL): Clear the main sequencer interrupt flag first! ---
    // If we don't clear this, the ISR will re-enter immediately.
    ADCIntClear(ADC0_BASE, 0);

    // Get the masked interrupt status for the comparators
    ui32CompStatus = ADCComparatorIntStatus(ADC0_BASE);

    // Clear the comparator interrupt flags
    ADCComparatorIntClear(ADC0_BASE, ui32CompStatus);

    // Get the data from the FIFO.
    ADCSequenceDataGet(ADC0_BASE, 0, pui32Data);

    // If a callback is registered, call it for each triggered comparator
    if (g_ADCCallback != 0)
    {
        // Check if Comparator 0 (AIN0) triggered the interrupt
        if (ui32CompStatus & (1 << 0))
        {
            g_ADCCallback(0, pui32Data[0]);
        }
        // Check if Comparator 1 (AIN1) triggered the interrupt
        if (ui32CompStatus & (1 << 1))
        {
            g_ADCCallback(1, pui32Data[1]);
        }
    }
}


void ADCTrigger_Init(uint32_t channel0_limit, uint32_t channel1_limit)
{
    // --- 1. Enable Peripherals ---
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // --- 2. Configure GPIO Pins ---
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    // --- 3. Configure ADC Sequencer & Comparators ---
    ADCSequenceDisable(ADC0_BASE, 0);
    // Configure SS0 to be triggered continuously by the ADC itself
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);

    // Step 0: AIN0 (PE3), use Comp 0
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_CMP0);
    // Step 1: AIN1 (PE2), use Comp 1, mark as end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1 | ADC_CTL_CMP1 | ADC_CTL_END);

    // Configure Comparator 0 for AIN0
    // We are only using the High Reference for the threshold (0 to channel0_limit is the Low Band)
    ADCComparatorRegionSet(ADC0_BASE, 0, 0, channel0_limit);

    // --- FIXED: Configure Comparator 1 for AIN1 ---
    ADCComparatorRegionSet(ADC0_BASE, 1, 0, channel1_limit);

    // --- FIXED: Tell comparators WHEN to fire an interrupt ---
    // Use ADC_COMP_INT_HIGH_ONCE: Interrupt once when value transitions into the High Band (> limit)
    // ADC_COMP_TRIG_NONE: We only want an interrupt, not a PWM/DMA trigger.
    ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_INT_HIGH_ONCE | ADC_COMP_TRIG_NONE);
    ADCComparatorConfigure(ADC0_BASE, 1, ADC_COMP_INT_HIGH_ONCE | ADC_COMP_TRIG_NONE);

    // --- 4. Enable Sequence and Interrupts ---

    // Re-enable the sequencer so it starts converting
    ADCSequenceEnable(ADC0_BASE, 0);

    // Clear any stale interrupt flags
    ADCIntClear(ADC0_BASE, 0);

    // Enable the interrupt at the peripheral (ADC) level
    ADCIntEnable(ADC0_BASE, 0);

    // Enable the interrupt at the controller (NVIC) level
    IntEnable(INT_ADC0SS0);
}

void ADCTrigger_SetCallback(ADCCallback callback)
{
    g_ADCCallback = callback;
}


// --- NEW FUNCTION IMPLEMENTATION ---

bool ADC_GetRealTimeValue(uint32_t channel, uint32_t *pui32Value)
{
    uint32_t pui32Data[2];

    // Check for a valid channel
    if (channel > 1)
    {
        return false;
    }

    // Read the latest data from the FIFO.
    // Since the sequence is running continuously (ADC_TRIGGER_ALWAYS),
    // this reads the most recently completed conversion results.
    // Note: We use ADCSequenceDataGet *without* manually triggering the ADC,
    // as it is already being triggered by the ADC_TRIGGER_ALWAYS setting.
    ADCSequenceDataGet(ADC0_BASE, 0, pui32Data);

    // Store the requested channel's value
    *pui32Value = pui32Data[channel];

    return true;
}
