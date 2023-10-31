#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "WS2812.pio.h" // This header file gets produced during compilation from the WS2812.pio file

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} RGBColor;

class Leds {
    public:
        Leds(PIO, int, int);
        void setColor_rgb(int, uint8_t, uint8_t, uint8_t);
        void setColor_rgb_mask(int*, uint8_t, uint8_t, uint8_t);
        void is_changed();
        void status();
        RGBColor get_colour_components_u8(uint32_t);
        void commit();
        void off();
        void setColor_hex(int, uint32_t);
 
    private:
        int sm;
        PIO _pio;
        int _size;
        uint32_t* leds;
        uint32_t* leds_to_update;

};

static inline uint32_t rgb_u32(uint8_t red, uint8_t green, uint8_t blue) {
    return ((uint32_t)(red) << 24)|((uint32_t)(green) << 16)|((uint32_t)(blue) << 8);
} 