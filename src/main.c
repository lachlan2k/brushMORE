#include <avr/io.h>
#include <util/delay.h>

// void set_a_high(bool on) {

// }

// void set_a_low(bool on) {

// }

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define AH_BIT 3
#define AL_BIT 2
#define AH_PORT PORTD
#define AL_PORT PORTD

#define CH_BIT 5
#define CL_BIT 1
#define CH_PORT PORTD
#define CL_PORT PORTB

#define ah AH_PORT, AH_BIT
#define al AL_PORT, AL_BIT
#define ch CH_PORT, CH_BIT
#define cl CL_PORT, CL_BIT


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

void close() {
    bitClear(AH_PORT, AH_BIT);
    bitClear(AL_PORT, AL_BIT);
    bitClear(CH_PORT, CH_BIT);
    bitClear(CL_PORT, CL_BIT);
}

void fwd() {
    close();
    bitSet(AH_PORT, AH_BIT);
    bitSet(CL_PORT, CL_BIT);
}

void rev() {
    close();
    bitSet(CH_PORT, CH_BIT);
    bitSet(AL_PORT, AL_BIT);
}

int main() {
    
    while (1) {
        fwd();

        green_off();
        red_on();
        
        _delay_ms(2000);

        rev();

        red_off();
        green_on();

        _delay_ms(2000);

        close();

        red_off();
        green_off();

        _delay_ms(2000);
    }
    
    return 0;
}