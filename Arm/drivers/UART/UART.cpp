#include <USB_CDC.h>

UART::UART(uart_inst_t *uart_num, uint baud_rate, uint tx_pin, uint rx_pin) {
        uart_ = uart_num;
        baud_rate_ = baud_rate;
        tx_pin_ = tx_pin;
        rx_pin_ = rx_pin;
        init();
    }

void init() {
    uart_init(uart_, baud_rate_);
    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);
    uart_set_hw_flow(uart_, false, false);
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
}

void send(const char* data) {
    uart_puts(uart_, data);
}

void receiveUntil(char* buffer, char terminator, size_t max_size) {
        size_t index = 0;
        char received_char;
        
        while (index < max_size - 1) {
            received_char = uart_getc(uart_);
            buffer[index] = received_char;
            index++;

            if (received_char == terminator) {
                break;
            }
        }
        
        // Null-terminate the received string
        buffer[index] = '\0';
    }
