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

#define READ_RC() ((RC_IN_PIN & _BV(RC_IN_BIT)) ? 1 : 0)

void io_init() {
    DDRB &= ~(_BV(RC_IN_BIT));
    // PORTB |= _BV(RC_IN_BIT);

    DDRC |= (1 << 2) | (1 << 3);
    GREEN_OFF();
    RED_OFF();
}

void icp_timer_init() {
    // enable icp interrupt and overflow interrupt
    TIMSK |= _BV(TICIE1) | _BV(TOIE1);

    // set the interrupt capture edge to high, and start the timer with no pre-scaler
    TCCR1B |=  _BV(ICES1) | _BV(CS10);
}

uint8_t overflow_count = 0;

void set_duty(uint8_t);

ISR (TIMER1_CAPT_vect) {
    // 1 = waiting for high, 0 = waiting for low
    static uint8_t edge = 1;

    if (edge) {
        TCNT1 = 0;
        TCCR1B &= ~_BV(ICES1);
    } else {
        TCCR1B |= _BV(ICES1);
        
        uint32_t time = TCNT1;
        uint32_t duty = 0;

        if (time < 17000 || time > 33000) {
            duty = 0;
        } else if (time > 32000) {
            duty = 255;
        } else if (time <= 16000) {
            duty = 0;
        } else {
            duty = 255 * (time - 16000) / 16000;
        }

        set_duty(duty);
    }

    edge = !edge;
    overflow_count = 0;
}

ISR (TIMER1_OVF_vect) {
    set_duty(0);
}

void pwm_timer_init() {
    // enable interrupt for overflow and comp match on timer 2 (For output pwm)
    TIMSK |= _BV(TOIE2) | _BV(OCIE2);

    // start timer 2 with no prescaler
    TCCR2 |= _BV(CS20);
}

// output pwm on
ISR (TIMER2_OVF_vect) {
    if (OCR2) RED_ON();
}

// output pwn off
ISR (TIMER2_COMP_vect) {
    RED_OFF();
}

void set_duty(uint8_t duty) {
    OCR2 = duty;
}

int main() {
    io_init();
    icp_timer_init();
    pwm_timer_init();

    sei();

    set_duty(0);

    while (1) {}
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