#ifndef DRIVERS_ADC_TRIGGER_H_
#define DRIVERS_ADC_TRIGGER_H_

#include <stdint.h>

/**
 * @brief Defines the function signature for the interrupt callback.
 * * @param channel The ADC channel that triggered the interrupt (0 or 1 in this driver).
 * @param value The 12-bit ADC value that was read.
 */
typedef void (*ADCCallback)(uint32_t channel, uint32_t value);

/**
 * @brief Initializes ADC0, Sequencer 0 to read AIN0 (PE3) and AIN1 (PE2).
 *
 * This function configures the ADC to run continuously and uses the digital
 * comparators (DCMP0 and DCMP1) to trigger an interrupt only when the
 * reading from AIN0 rises above channel0_limit or AIN1 rises above
 * channel1_limit.
 *
 * @param channel0_limit The 12-bit (0-4095) threshold for AIN0.
 * @param channel1_limit The 12-bit (0-4095) threshold for AIN1.
 */
void ADCTrigger_Init(uint32_t channel0_limit, uint32_t channel1_limit);

/**
 * @brief Registers a callback function to be executed inside the ADC ISR.
 *
 * @param callback The function pointer to your callback function.
 */
void ADCTrigger_SetCallback(ADCCallback callback);


#endif /* DRIVERS_ADC_TRIGGER_H_ */
