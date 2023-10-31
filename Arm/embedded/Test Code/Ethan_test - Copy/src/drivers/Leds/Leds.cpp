#include "drivers/logging/logging.h"
// Include any additional libraries needed for your LED strip.
#include "Leds.h"
#include <stdexcept>
#include <iostream>



Leds::Leds(PIO pio, int LED_PIN, int _size) {
    this-> _pio = pio;
    this-> sm = 0;

    // if (_size <= 0) {
    //     // Handle the error, e.g., throw an exception or log an error message
    //     throw std::invalid_argument("Size must be greater than 0.");
    // }

    this-> _size = _size;
    this->leds = new uint32_t[_size];
    this->leds_to_update = new uint32_t[_size];
    uint pio_program_offset = pio_add_program(pio, &ws2812_program);    
    ws2812_program_init(pio, 0, pio_program_offset, LED_PIN, 800000, false);
    for(int i = 0; i < _size; i++){
        leds[i] = 0;
    }
    for(int i = 0; i < _size; i++){
        leds_to_update[i] = 0;
    }
}

void Leds::setColor_rgb(int LED, uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t colour = rgb_u32(red, green, blue);
    leds_to_update[LED] = colour;
    std::string logMessage = "Setting LED " + std::to_string(LED) + " color to RGB(" + std::to_string(red) + ", " + std::to_string(green) + ", " + std::to_string(blue) + ")";
    log(LogLevel::INFORMATION, logMessage.c_str());
}

void Leds::setColor_hex(int LED, uint32_t colour) {
    leds_to_update[LED] = colour;
    std::string logMessage = "Setting LED " + std::to_string(LED) + " color to " + std::to_string(colour);
    log(LogLevel::INFORMATION, logMessage.c_str());
}

void Leds::setColor_rgb_mask(int* mask, uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t colour = rgb_u32(red, green, blue);
    for(int i = 0; i < _size; i++){
        if(mask[i] == 1){
            leds_to_update[i] = colour;
        }
    }
    log(LogLevel::INFORMATION, ("Setting LED mask color to RGB(" + std::to_string((int)red) + ", " + std::to_string((int)green) + ", " + std::to_string((int)blue) + ")").c_str());
}

void Leds::off() {
    // Turn off the LED strip.
    // Initialize all elements to 0.
    for(int i = 0; i < _size; i++){
        leds[i] = 0;
    }
    for(int i = 0; i < _size; i++){
        leds_to_update[i] = 0;
    }
    commit();
    log(LogLevel::INFORMATION,"Turning off the LED strip.");
}

void Leds::commit() {
    for (int i = 0; i < _size; i++) {
        leds[i] = leds_to_update[i];
        pio_sm_put_blocking(_pio, sm, leds[i]);
    }
}

RGBColor Leds::get_colour_components_u8(uint32_t colour) {
// Extract the individual color components
    RGBColor components;
    components.red = (colour >> 16) & 0xFF;
    components.green = (colour >> 8) & 0xFF;
    components.blue = colour & 0xFF;
    return(components);
}

void Leds::status() {
    for (int i = 0; i < _size; i++) {
        RGBColor rgb = get_colour_components_u8(leds[i]);
        bool is_on = rgb.red > 0 | rgb.green > 0 | rgb.blue > 0;
        log(LogLevel::INFORMATION, ("LED: " + std::to_string((int)i) + " Status:" + std::to_string((bool)is_on) + " " + std::to_string((int)rgb.red) + ", " + std::to_string((int)rgb.green) + ", " + std::to_string((int)rgb.blue)).c_str());
    
    }
}

void Leds::is_changed() {
    for (int i = 0; i < _size; i++) {
        bool is_changed = leds[i] != leds_to_update[i];
        log(LogLevel::INFORMATION, ("LED " + std::to_string(i) + "changed status: " + std::to_string(is_changed)).c_str());
    }
}
