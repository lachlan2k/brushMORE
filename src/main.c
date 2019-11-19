#include <avr/io.h>
#include <util/delay.h>

void red_on() {
    DDRC |= (1 << 3);
}

void red_off() {
    DDRC &= ~(1 << 3);
}

void green_on() {
    DDRC |= (1 << 2);
}

void green_off() {
    DDRC &= ~(1 << 2);
}

int main() {
    while (1) {
        green_off();
        red_on();
        _delay_ms(500);
        red_off();
        green_on();
        _delay_ms(500);
    }
    
    return 0;
}