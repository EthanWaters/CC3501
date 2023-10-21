#include "Microphone.h"

Microphone::Microphone(int ADC_PIN) {
    this-> _ADC_PIN = ADC_PIN;
    
    adc_init();
    adc_gpio_init(_ADC_PIN);
    adc_select_input(_ADC_PIN);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        false,    // Enable DMA data request (DREQ)
        0,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8-bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
    adc_set_clkdiv(0);

}

void read(uint16_t* buffer, uint8_t capture_depth) {

    adc_run(true);
    for (uint8_t i = 0; i < capture_depth; ++i) {
        buffer[i] =  adc_fifo_get_blocking();
    }
    
    // Disable free-running mode
    adc_run(false);

    adc_fifo_drain();
    
}