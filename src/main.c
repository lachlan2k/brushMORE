#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// void set_a_high(bool on) {

// }

// void set_a_low(bool on) {

// }

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define AH_BIT PD3
#define AL_BIT PD2
#define AH_PORT PORTD
#define AL_PORT PORTD

#define CH_BIT PD5
#define CL_BIT PB1
#define CH_PORT PORTD
#define CL_PORT PORTB

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

void set_duty(unsigned int duty) {
    OCR1B = duty;
}

int main() {
    TIMSK |= _BV(TOIE1) | _BV(OCIE1B);
    TCCR1B |= _BV(CS00);

    sei();

    while (1) {
        for(unsigned int i = 0; i < 40000; i += 10) {
            set_duty(i);
            _delay_ms(1);
        }
        for(unsigned int i = 39500; i > 0; i -= 10) {
            set_duty(i);
            _delay_ms(1);
        }
    }
}

ISR (TIMER1_OVF_vect) {
    red_on();
}

ISR (TIMER1_COMPB_vect) {
    red_off();
}

// void __attribute__ ((noinline)) close() {
//     bitClear(AH_PORT, AH_BIT);
//     bitClear(AL_PORT, AL_BIT);
//     bitClear(CH_PORT, CH_BIT);
//     bitClear(CL_PORT, CL_BIT);
// }

// void fwd() {
//     close();
//     bitSet(AH_PORT, AH_BIT);
//     bitSet(CL_PORT, CL_BIT);
// }

// void rev() {
//     close();
//     bitSet(CH_PORT, CH_BIT);
//     bitSet(AL_PORT, AL_BIT);
// }

// void ioinit() {
//     PORTC = 0;
//     DDRC = _BV(PC2) & _BV(PC3); // TODOO FIXXX
// }

// int main() {
//     ioinit();
//     _delay_ms(5000);
//     PORTC |= _BV(PC2);
//     _delay_ms(1000);
//     PORTC &= ~_BV(PC2);
//     while (1);
// }

// int main() {
//     while (1) {
//         // fwd();
//         bitSet(AH_PORT, AH_BIT);

//         // green_off();
//         // red_on();
        
//         // _delay_ms(2000);
//         bitClear(AH_PORT, AH_BIT);

//         // rev();

//         // red_off();
//         // green_on();

//         // _delay_ms(2000);

//         // close();

//         // red_off();
//         // green_off();

//         _delay_ms(2000);
//     }
// }