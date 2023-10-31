#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "WS2812.pio.h" // This header file gets produced during compilation from the WS2812.pio file
#include <stdexcept>
#include <iostream>
#include "hardware/adc.h"
#include "hardware/pio.h"

#define CAPTURE_CHANNEL 0
#define CAPTURE_DEPTH 1000


class Microphone {
    public:
        Microphone(int ADC_PIN);
        void read(uint16_t* buffer, uint8_t capture_depth);
 
    private:
        int _ADC_PIN;

};