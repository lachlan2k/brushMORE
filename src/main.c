#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

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

#define RED_LED_BIT PC3
#define RED_LED_PORT PORTC

#define GREEN_LED_BIT PC2
#define GREEN_LED_PORT PORTC

#define RC_IN_BIT PB0
#define RC_IN_PIN PINB

#define RED_ON() (RED_LED_PORT &= ~(1 << RED_LED_BIT))
#define RED_OFF() (RED_LED_PORT |= (1 << RED_LED_BIT))
#define GREEN_ON() (GREEN_LED_PORT &= ~(1 << GREEN_LED_BIT))
#define GREEN_OFF() (GREEN_LED_PORT |= (1 << GREEN_LED_BIT))

void io_init() {
    DDRB &= ~(_BV(RC_IN_BIT));
    PORTB |= _BV(RC_IN_BIT);

    DDRC |= (1 << 2) | (1 << 3);
    GREEN_OFF();
    RED_OFF();
}

void timer_init() {
    // enable interrupt for compa match
    TIMSK |= _BV(OCIE1A);

     // TODO: calculate based on desired pwm frequency
    // (0.000001/(1/16000000))-1 = 15, 1 microsecond period
    OCR1A = 10;
    OCR1B = 10;

    // start timer 1 with no prescaler, and enable CTC
    TCCR1B |= _BV(CS00) | _BV(WGM12);
}

inline int read_rc() {
    if (RC_IN_PIN & _BV(RC_IN_BIT))
        return 1;
    else
        return 0;
}

int main() {
    io_init();
    timer_init();

    sei();

    while (1) {}
}

enum RCInState {
    RC_IN_ERROR,
    RC_IN_WAITING_FOR_SIGNAL,
    RC_IN_NORMAL
};

volatile struct {
    enum RCInState state;
    uint16_t value;
} rc_in = {
    .value = 0,
    .state = RC_IN_WAITING_FOR_SIGNAL
};

#define MIN_HIGH 500
#define MAX_HIGH 2500

#define HIGH_FLOOR 1000
#define HIGH_CEIL 2000

// #define MAX_LOW 50000
#define MAX_LOW 1000000

void update_input() {
    static uint16_t high_count = 0;
    static uint32_t low_count = 0;

    int is_high = !read_rc();

    if (is_high) {
        // reset low count for next pulse
        if (low_count)
            low_count = 0;

        if (high_count < MAX_HIGH)
            high_count++;
        else
            rc_in.state = RC_IN_ERROR;
    } else {
        // transition from high to low
        if (high_count > 0) {
            // pulse was too short
            if (high_count < MIN_HIGH)
                rc_in.state = RC_IN_ERROR;
            else {
                // pulse was normal, bound to [HIGH_FLOOR, HIGH_CEIL] and set
                if (high_count <= HIGH_FLOOR)
                    rc_in.value = HIGH_FLOOR;
                else if (high_count >= HIGH_CEIL)
                    rc_in.value = HIGH_CEIL;
                else
                    rc_in.value = high_count;
            }

            // reset high count for next pulse
            high_count = 0;
        }

        if (low_count < MAX_LOW)
            low_count++;
        else
            rc_in.state = RC_IN_WAITING_FOR_SIGNAL;
    }
}

void update_output() {
    switch (rc_in.state) {
        case RC_IN_ERROR:
            RED_ON();
            GREEN_OFF();
            break;

        case RC_IN_NORMAL:
            RED_OFF();
            GREEN_OFF();
            break;

        case RC_IN_WAITING_FOR_SIGNAL:
            RED_OFF();
            GREEN_ON();
            break;
    }
}

uint32_t x = 0;

ISR (TIMER1_COMPA_vect) {
    if (++x > 1000000) {
        GREEN_ON();
        RED_OFF();
    } else {
        RED_ON();
        GREEN_OFF();
    }

    // update_input();
    // update_output();
}

// ISR (TIMER1_OVF_vect) {
//     red_on();
// }

// ISR (TIMER1_COMPB_vect) {
//     red_off();
// }

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